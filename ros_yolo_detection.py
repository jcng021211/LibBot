#!/usr/bin/env python3

import time
import threading

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyResponse
from ultralytics import YOLO


#Constrains
MODEL_PATH = '/home/jcng/best.pt'
CONFIDENCE_THRESHOLD = 0.75
IOU_THRESHOLD = 0.8
IOU_COOLDOWN = 10
AVOID_COOLDOWN = 3
AREA_COOLDOWN = 20
PAUSE_DURATION = 6
BACKUP_SPEED = -0.3
BACKUP_DURATION = 0.6
TURN_SPEED = 0.6
TURN_DURATION = 1.5
FORWARD_SPEED = 0.5
FORWARD_DURATION = 1.5
MANEUVER_TIME = BACKUP_DURATION + TURN_DURATION + FORWARD_DURATION + 0.6
AVOID_COOLDOWN = max(AVOID_COOLDOWN, MANEUVER_TIME + 0.5)


class YoloAvoidanceNode:
    def __init__(self):
        rospy.init_node('yolo8_turtlebot_detection')
        self.model = YOLO(MODEL_PATH)
        self.bridge = CvBridge()
        
        # Display/publish options
        self.show_window = rospy.get_param("~show_window", True)
        self.publish_annotated = rospy.get_param("~publish_annotated", True)
        
        # Window controls
        self.win_name = rospy.get_param("~win_name", "YOLOv8 Detection")
        self.win_width = int(rospy.get_param("~win_width", 640))
        self.win_height = int(rospy.get_param("~win_height", 360))
        self._win_made = False

        # Publishers
        self.detection_pub = rospy.Publisher('/object_detection_event', String, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pause_pub = rospy.Publisher('/pause_detection', String, queue_size=10)

        self.avoiding = False
        
        # Subscribers
        rospy.Subscriber('/nav_phase', String, self.nav_phase_callback)
        rospy.Subscriber('/pause_detection', String, self.pause_callback)
        rospy.Subscriber('/raspicam_node_1/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber('/last_goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # State
        self.nav_phase = "idle"
        self.detection_paused_until = 0
        self.last_avoid_time = 0
        self.last_goal = None
        self.last_avoid_position = None
        self.current_pos = None
        self.last_amcl_time = 0.0
        self.detected_area_cache = {}
        self.detection_disabled = False
        # ---- Spatial “one-and-done” flags (map/odom frame) ----
        self.flag_radius = rospy.get_param("~flag_radius", 0.20)
        self.flag_ttl    = rospy.get_param("~flag_ttl", 0)
        self.flagged_areas = []
        self.flag_lock = threading.Lock()
        # re-trigger guards
        self.retrigger_min_dist = rospy.get_param("~retrigger_min_dist",
                                                  max(0.6, self.flag_radius*2))
        self.retrigger_grace    = rospy.get_param("~retrigger_grace", 2.0)
        
        self.infer_period = rospy.get_param("~infer_period", 0.20)
        self.last_infer = 0.0
        self.show_period = rospy.get_param("~show_period", 0.10)
        self.last_show = 0.0

        # clear remembered areas on demand
        rospy.Service('~clear_flags', Empty, self._srv_clear_flags)
        
        # Annotated image publisher (view with rqt_image_view)
        self.ann_pub = rospy.Publisher('/yolo_annotated/compressed', CompressedImage, queue_size=1)
 
        rospy.loginfo("YOLOv8 node started.")
        
    def _ensure_window(self):
        if not self.show_window or self._win_made:
            return
        try:
            cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name, self.win_width, self.win_height)
        except Exception:
            pass
        self._win_made = True

    def _srv_clear_flags(self, _req):
        with self.flag_lock:
            self.flagged_areas = []
        rospy.loginfo("[FLAG] Cleared all flagged areas.")
        return EmptyResponse()

    def nav_phase_callback(self, msg):
        """Update navigation phase."""
        self.nav_phase = msg.data

    def pause_callback(self, msg):
        """Pause detections briefly."""
        self.detection_paused_until = time.time() + PAUSE_DURATION

    def goal_callback(self, msg):
        """Store the last sent goal."""
        rospy.loginfo("[AVOID] Received new goal.")
        self.last_goal = msg
        
    def pose_callback(self, msg):
        """Track current robot position from AMCL."""
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_amcl_time = time.time()
        
    def is_near_goal(self):
        """Check if the robot is near the goal."""
        if self.last_goal is None or self.current_pos is None:
            return False
        cx, cy = self.current_pos
        gx, gy = self.last_goal.pose.position.x, self.last_goal.pose.position.y
        dist = np.hypot(cx - gx, cy - gy)
        return dist < 0.5
        
    # ---------- Flag helpers ----------
    def _near_point(self, p1, p2, r):
        return np.hypot(p1[0] - p2[0], p1[1] - p2[1]) <= r
    
    def _canonical_label(self, label: str) -> str:
        # normalize labels so "foo (flagged)" and "foo (flagged) (flagged)" map to "foo"
        base = label
        while base.endswith(" (flagged)"):
            base = base[: -len(" (flagged)")]
        return base

    def _flag_current_area(self, label):
        """Remember current position as a flagged area for this label (no re-detect/log here)."""
        if self.current_pos is None:
            return
        label = self._canonical_label(label)
        now = time.time()
        with self.flag_lock:
            # Merge if very close to an existing flag for same label
            for fa in self.flagged_areas:
                if fa['label'] == label and self._near_point(self.current_pos, fa['pos'], self.flag_radius):
                    fa['time'] = now
                    return
            self.flagged_areas.append({'label': label, 'pos': self.current_pos, 'time': now})
        rospy.loginfo(f"[FLAG] Marked '{label}' area at ({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) ±{self.flag_radius:.2f} m")

    def _in_flagged_area(self):
        """Return a flag dict if we're inside any (non-expired) flagged area; else None."""
        if self.current_pos is None:
            return None
        now = time.time()
        keep, found = [], None
        with self.flag_lock:
            for fa in self.flagged_areas:
                if self.flag_ttl > 0 and (now - fa['time']) > self.flag_ttl:
                    continue
                keep.append(fa)
                if found is None and self._near_point(self.current_pos, fa['pos'], self.flag_radius):
                    found = fa
            self.flagged_areas = keep
        return found

    def _update_detection_gate(self):
        """Disable only while approaching shelf and within the near-goal radius."""
        disable = (self.nav_phase == "to_shelf" and self.is_near_goal())

        # Transition -> disabled
        if disable and not self.detection_disabled:
            self.detection_disabled = True
            rospy.loginfo("[DETECT] Paused near shelf goal.")
            self.pause_pub.publish('pause')
            self.detection_pub.publish("[DETECT] Paused near shelf goal.")
            return

        # Transition -> enabled (leaving shelf zone OR any other phase)
        if not disable and self.detection_disabled:
            self.detection_disabled = False
            rospy.loginfo(f"[DETECT] Gate reopened (phase={self.nav_phase}).")
            self.detection_pub.publish("[DETECT] Resumed.")

    def image_callback(self, msg):
        # Update the gate first, every frame
        self._update_detection_gate()
        
        now = time.time()
        
        # always decode the camera frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame_to_show = cv_image.copy()
        if self.detection_disabled:
            cv2.putText(frame_to_show, "DETECT PAUSED", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        
        # First: if inside a flagged area, skip YOLO and just avoid (cooldown/nav-phase/avoiding guarded)
        fa = self._in_flagged_area()
        # If you're effectively at the shelf goal, skip avoidance from flags too
        if self.nav_phase == "to_shelf" and self.is_near_goal():
            fa = None
        if (fa and not self.avoiding and self.nav_phase in ["to_shelf", "to_rest"]):
            
            # short grace period right after a maneuver
            if (now - self.last_avoid_time) < self.retrigger_grace:
                return
            # require that we moved out of the last avoidance neighborhood
            if self.last_avoid_position and self.current_pos:
                if np.hypot(self.current_pos[0]-self.last_avoid_position[0],
                            self.current_pos[1]-self.last_avoid_position[1]) < self.retrigger_min_dist:
                    return
            base_label = self._canonical_label(fa['label'])
            rospy.loginfo(f"[AVOID] Entering flagged area for '{base_label}' → skipping detection and executing avoidance.")
            # start cooldown immediately to debounce logs until the maneuver thread finishes
            self.last_avoid_time = now
            self.trigger_avoidance(base_label)
            # show/publish the current (raw) frame anyway so the window stays live
            if self.show_window and (now - self.last_show >= self.show_period):
                self.last_show = now
                self._ensure_window()
                try:
                    cv2.imshow(self.win_name, frame_to_show)
                    cv2.waitKey(1)
                except Exception:
                    pass

            if self.publish_annotated:
                try:
                    out = CompressedImage()
                    out.header.stamp = rospy.Time.now()
                    out.format = "jpeg"
                    out.data = np.array(cv2.imencode('.jpg', frame_to_show)[1]).tobytes()
                    self.ann_pub.publish(out)
                except Exception:
                    pass
            return
            
        # Decide whether to run YOLO; ALWAYS show/publish a frame below
        should_run_yolo = (
            (not self.detection_disabled)
            and (now >= self.detection_paused_until)
            and (self.nav_phase != "idle")
            and (now - self.last_infer >= self.infer_period)  # <— add
        )

        if should_run_yolo:
            self.last_infer = now
            try:
                results = self.model(cv_image, imgsz=416, verbose=False)

                if results:
                    annotated = results[0].plot()
                    frame_to_show = annotated

                    for r in results:
                        for box in r.boxes:
                            label = self.model.names[int(box.cls[0])]
                            conf = float(box.conf[0])
                            if conf < CONFIDENCE_THRESHOLD:
                                continue

                            if not (label.startswith("book_") or label == "rubbish"):
                                continue

                            coords = tuple(box.xyxy[0].tolist())
                            now = time.time()

                            last_coords, last_seen, last_conf = self.detected_area_cache.get(label, ((0, 0, 0, 0), 0, 0.0))

                            if compute_iou(coords, last_coords) > IOU_THRESHOLD:
                        
                                if now - last_seen < AREA_COOLDOWN:  # Same area and recent detection
                                    continue
                                elif conf <= last_conf:
                                    continue
                                else:
                                    self.log_detection(label, conf, coords)
                                    self.detected_area_cache[label] = (coords, now, conf)
                            else:
                                # New area — always log
                                self.log_detection(label, conf, coords)
                                self.detected_area_cache[label] = (coords, now, conf)

                            # react immediately if we’re allowed to
                            if (not self.avoiding) and (now - self.last_avoid_time > AVOID_COOLDOWN):
                                self.trigger_avoidance(label)

                            # --- keep your logging/area-cache logic strictly for logging de-dup ---
                            last_coords, last_seen, last_conf = self.detected_area_cache.get(label, ((0, 0, 0, 0), 0, 0.0))
                            if compute_iou(coords, last_coords) > IOU_THRESHOLD:
                                if now - last_seen < AREA_COOLDOWN:
                                    continue
                                elif conf <= last_conf:
                                    continue
                                else:
                                    self.log_detection(label, conf, coords)
                                    self.detected_area_cache[label] = (coords, now, conf)
                            else:
                                self.log_detection(label, conf, coords)
                                self.detected_area_cache[label] = (coords, now, conf)

            except Exception as e:
                rospy.logerr(f"YOLOv8 inference failed: {e}")
                
        # Show/publish exactly once per callback (raw or annotated)
        if self.show_window:
            self._ensure_window()
            try:
                cv2.imshow(self.win_name, frame_to_show)
                cv2.waitKey(1)
            except Exception:
                pass

        if self.publish_annotated:
            try:
                out = CompressedImage()
                out.header.stamp = rospy.Time.now()
                out.format = "jpeg"
                out.data = np.array(cv2.imencode('.jpg', frame_to_show)[1]).tobytes()
                self.ann_pub.publish(out)
            except Exception:
                pass

    def log_detection(self, label, conf, coords):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        x1, y1, x2, y2 = coords
        msg = (f"[{timestamp}] Detected: {label} ({conf*100:.1f}%) "
               f"at [x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}]")
        rospy.loginfo(msg)
        with open("detection_log.txt", "a") as f:
            f.write(msg + "\n")

        # Publish simplified message (no confidence)
        simple_msg = f"Found {label} at ({int(x1)}, {int(y1)})"
        self.detection_pub.publish(simple_msg)

    def trigger_avoidance(self, label):
    
        if self.avoiding:
            return
            
        # do not re-trigger if we haven't left the previous avoidance neighborhood
        if self.last_avoid_position and self.current_pos:
            if np.hypot(self.current_pos[0]-self.last_avoid_position[0],
                        self.current_pos[1]-self.last_avoid_position[1]) < self.retrigger_min_dist:
                return
      
        self.avoiding = True

        """Cancel goal, stop robot, and start avoidance maneuver."""
        self.cancel_pub.publish(GoalID())
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.2)
        self._publish_for_duration(Twist(), 0.4, rate_hz=30)
        
        rospy.loginfo(f"[AVOID] {label} detected. Executing reverse-turn.")
        self.detection_pub.publish(f"[AVOID] {label} detected. Executing reverse-turn.")
        
        self.pause_pub.publish('pause')
        
        # Remember this place so we won't re-detect/log the same object at the same area next time
        self._flag_current_area(label)
        # start cooldown right away (we only set last_avoid_position in finally)
        self.last_avoid_time = time.time()
     
        threading.Thread(target=self.perform_avoidance_maneuver, daemon=True).start()
        
    def _publish_for_duration(self, twist, duration, rate_hz=20):
        start_time = time.time()
        rate = rospy.Rate(rate_hz)  # 10 Hz
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def perform_avoidance_maneuver(self):
                         
        if self.last_avoid_position and self.current_pos:
            dist = np.hypot(self.current_pos[0] - self.last_avoid_position[0],
                            self.current_pos[1] - self.last_avoid_position[1])

            if dist < 0.5 and time.time() - self.last_avoid_time < AREA_COOLDOWN:
                self.avoiding = False  # clear the latch before returning
                return  # Skip if too close to previous avoidance position

        try:
            # Reverse
            twist = Twist()
            twist.linear.x = BACKUP_SPEED
            self._publish_for_duration(twist, BACKUP_DURATION, rate_hz=20)

            # Turn
            twist = Twist()
            twist.angular.z = TURN_SPEED
            self._publish_for_duration(twist, TURN_DURATION)

            # Forward
            twist = Twist()
            twist.linear.x = FORWARD_SPEED
            self._publish_for_duration(twist, FORWARD_DURATION)
            
            # larger opposite turn
            t = Twist(); t.angular.z = -TURN_SPEED
            self._publish_for_duration(t, TURN_DURATION)

            # longer forward
            t = Twist(); t.linear.x = FORWARD_SPEED
            self._publish_for_duration(t, FORWARD_DURATION)

            # Re-send goal (after making sure AMCL is fresh and costmaps are sane)
            # 1) wait briefly for a fresh amcl update (<=0.5s old)
            start = time.time()
            while time.time() - start < 2.0:
                if self.last_amcl_time and (time.time() - self.last_amcl_time) <= 0.5:
                    break
                rospy.sleep(0.05)

            # 2) clear costmaps (optional but helps if planner balked)
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
                rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
            except Exception as e:
                rospy.logwarn(f"[AVOID] clear_costmaps failed: {e}")

            # 3) publish the goal again (fresh stamp, only once)
            if self.last_goal and self.nav_phase in ["to_shelf", "to_rest"]:
                g = PoseStamped()
                g.header.frame_id = self.last_goal.header.frame_id or "map"
                g.header.stamp = rospy.Time.now()
                g.pose = self.last_goal.pose
                rospy.loginfo("[AVOID] Re-sending goal after avoidance")
                self.goal_pub.publish(g)
                resume = "[RESUME] Resuming navigation to goal." if self.nav_phase == "to_shelf" else "[RESUME] Resuming return to rest position."
                self.detection_pub.publish(resume)
        except Exception as e:
            rospy.logwarn(f"[AVOID] Motion error during reverse-turn: {e}")
            
        finally:
            # stamp completion so the next run can compare against THIS one
            self.last_avoid_time = time.time()
            if self.current_pos is not None:
                self.last_avoid_position = self.current_pos
            self.avoiding = False
     
def compute_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    inter_area = max(0, xB - xA) * max(0, yB - yA)

    boxA_area = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxB_area = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    return inter_area / float(boxA_area + boxB_area - inter_area + 1e-5)

if __name__ == '__main__':
    YoloAvoidanceNode()
    rospy.spin()
    cv2.destroyAllWindows()
