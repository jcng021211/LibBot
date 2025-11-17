#!/usr/bin/env python3

import os
import time
import json
import threading
import re
import logging
import tkinter as tk
from tkinter import messagebox, scrolledtext
from logging.handlers import RotatingFileHandler

import rospy
import rospkg
from PIL import Image, ImageTk
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from sound_play.libsoundplay import SoundClient
from turtlebot3_msgs.msg import Sound
import actionlib


# Constants
REST_COORD = dict(x=0.1, y=-0.19, z=-0.10830729764148193, w=0.9941174625151695)
BATTERY_THRESHOLD = 10  # percent
LOG_FILE = 'detection_log.txt'
PAUSE_DURATION = 6      # seconds

# Sound values
SOUND_ALERT = 1
SOUND_BEEP = 5
SOUND_END = 0

log_handler = RotatingFileHandler('detection_log.txt', maxBytes=5 * 1024 * 1024, backupCount=3)
logging.basicConfig(handlers=[log_handler], level=logging.INFO, format='%(asctime)s - %(message)s')

class TurtlebotUI:
    def __init__(self):
        # State
        self.battery_percentage = 100
        self.is_navigating = False
        self.navigation_phase = 'idle'
        self.last_log_time = None
        self.has_played_shelf_sound = False
        self.current_target_book = None
        self.nav_session_id = None  # current navigation session id string

        # ROS & Data
        self._init_ros()
        self.books_data = self._load_book_data()

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title('TurtleBot Smart Library Assistant')
        self.root.geometry('550x700')
        self._load_icons()
        self._build_ui()
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)

        # Periodic updates
        self._schedule_icon_update()

        # Start UI loop
        self.root.mainloop()

    def _init_ros(self):
        rospy.init_node('turtlebot_ui_goal_sender', anonymous=True)

        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.last_goal_pub = rospy.Publisher('/last_goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.nav_phase_pub = rospy.Publisher('/nav_phase', String, queue_size=10)
        self.sound_pub = rospy.Publisher('/sound', Sound, queue_size=10)
        self.soundhandle = SoundClient()
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('[INFO] Waiting for move_base server...')
        self.move_client.wait_for_server()
        rospy.loginfo('[INFO] Connected to move_base server.')

        # Subscribers
        rospy.Subscriber('/object_detection_event', String, self.detection_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.navigation_feedback_callback)

        # Run ROS spin in background
        threading.Thread(target=rospy.spin, daemon=True).start()
        
        
    def show_full_image(self, filename):
        # Open a new window to display the full-size image
        win = tk.Toplevel(self.root)
        win.title("Full Image View")
        img_obj = self._load_book_image(filename, size=(900, 600))  # Load the image in a larger size
        
        if img_obj:
            img_label = tk.Label(win, image=img_obj)
            img_label.image = img_obj
            img_label.pack()
        else:
            tk.Label(win, text="Image not found").pack()


    def _load_book_data(self):
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('beginner_tutorials')
            path = os.path.join(pkg_path, 'scripts', 'book_locations.json')
            with open(path) as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f'[ERROR] Loading book data failed: {e}')
            return []

    def _load_icons(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('beginner_tutorials')
        def load(file):
            img = Image.open(os.path.join(pkg_path, 'scripts', file)).resize((28,28), Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(img)
        self.default_icon = load('bell.png')
        self.new_icon = load('notification.png')
        
        
    def _load_book_image(self, filename, size=(90, 120)):

        try:
            if not os.path.isabs(filename):
                base_dir = os.path.dirname(__file__)
                filename = os.path.join(base_dir, filename)

            img = Image.open(filename).resize(size, Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(img)
        except Exception:
            return None
            
        
    def _log(self, text):
        """Append a single line to the shared LOG_FILE."""
        with open(LOG_FILE, 'a') as f:
            f.write(text + "\n")

    def _start_nav_section(self, book):
        """Begin a nicely formatted section for this navigation."""
        self.nav_session_id = time.strftime("%Y%m%d-%H%M%S")
        header = (
            "\n\n" +
            "=" * 72 + "\n" +
            f" NAVIGATION START  [{self.nav_session_id}]\n" +
            "-" * 72 + "\n" +
            f" Target : \"{book['title']}\"  (Call#: {book['call_number']})\n" +
            f" Where  : Zone {book['zone']}, Bay {book['bay']}, Shelf {book['shelf']} ({book['side']})\n" +
            f" Phase  : to_shelf\n" +
            "=" * 72
        )
        self._log(header)

    def _end_nav_section(self, outcome):
        """Close the section (outcome = 'Reached & back to rest', 'Aborted', etc.)."""
        if not self.nav_session_id:
            return
        footer = (
            "-" * 72 + "\n" +
            f" NAVIGATION END    [{self.nav_session_id}]  → {outcome}\n" +
            "=" * 72 + "\n"
        )
        self._log(footer)
        self.nav_session_id = None


    def _build_ui(self):
        self.buttons = []
    
        # Notification button
        self.notification_btn = tk.Button(self.root, image=self.default_icon, bd=0, command=self.view_log)
        self.notification_btn.has_new_log = False
        self.notification_btn.pack(pady=5)

        # Search entry
        tk.Label(self.root, text='Enter Book Title or Call Number:').pack()
        self.entry = tk.Entry(self.root, width=50)
        self.entry.pack()
        tk.Button(self.root, text='Search', command=self.search_book).pack(pady=5)

        # Results frame
        self.result_frame = tk.Frame(self.root)
        self.result_frame.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(self.result_frame)
        self.scrollbar = tk.Scrollbar(self.result_frame, orient='vertical', command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas)
        self.scrollable_frame.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.canvas.create_window((0,0), window=self.scrollable_frame, anchor='nw')
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.scrollbar.pack(side='right', fill='y')

        # Control buttons & status
        tk.Button(self.root, text='Reset', command=self.reset_ui).pack(pady=5)
        self.status_label = tk.Label(self.root, text='System ready.')
        self.status_label.pack(pady=10)
        self.battery_label = tk.Label(self.root, text='Battery: --%')
        self.battery_label.pack(pady=5)
        
        # Dedicated detection message label
        self.detection_label = tk.Label(
            self.root, 
            text='No detections yet.',
            wraplength=480,
            justify='left', 
            font=('Courier', 10),
            fg='red'
        )
        self.detection_label.pack(pady=5)

    def _schedule_icon_update(self):
        self.root.after(3000, self.update_icon_state)

    def update_icon_state(self):
        try:
            mtime = os.path.getmtime(LOG_FILE)
            if self.last_log_time is None:
                self.last_log_time = mtime
            if mtime > self.last_log_time:
                self.notification_btn.config(image=self.new_icon)
                self.notification_btn.has_new_log = True
            else:
                self.notification_btn.config(image=self.default_icon)
                self.notification_btn.has_new_log = False
        except FileNotFoundError:
            self.notification_btn.config(image=self.default_icon)
            self.notification_btn.has_new_log = False
        self._schedule_icon_update()

    def view_log(self):
        try:
            with open(LOG_FILE) as f:
                content = f.read()
            self.last_log_time = os.path.getmtime(LOG_FILE)
        except FileNotFoundError:
            content = 'No log file found.'
        self.notification_btn.config(image=self.default_icon)
        win = tk.Toplevel(self.root)
        win.title('Detection Log')
        text = scrolledtext.ScrolledText(win, wrap=tk.WORD, font=('Courier',10))
        text.insert(tk.END, content)
        text.config(state='disabled')
        text.pack(fill='both', expand=True)

    def reset_ui(self):
        self.entry.delete(0, tk.END)
        for w in self.scrollable_frame.winfo_children():
            w.destroy()
        self.status_label.config(text='System ready.')

    def search_book(self):
        query = self.entry.get().strip().lower()
        for child in self.scrollable_frame.winfo_children():
            child.destroy()
        
        # Case 1: empty or only spaces
        if not query:
            tk.Label(
                self.scrollable_frame, 
                text="Please enter a book title or call number.", 
                font=('Arial', 12, 'italic'),
                fg='red'
            ).pack(pady=10)
            return

        # Case 2: normal search
        found = False
        for book in self.books_data:
            if query in book['title'].lower() or query in book['call_number'].lower():
                self._create_book_card(book)
                found = True

        # Case 3: no matches found
        if not found:
            tk.Label(
                self.scrollable_frame, 
                text="No book found.", 
                font=('Arial', 12, 'italic'),
                fg='red'
            ).pack(pady=10)

    def _create_book_card(self, book):
        card = tk.Frame(self.scrollable_frame, bd=2, relief='groove', padx=10, pady=8)
        card.pack(fill='x', pady=6, padx=10)

        # 2-column grid: left = text, right = image + button
        card.grid_columnconfigure(0, weight=1)  # text grows
        card.grid_columnconfigure(1, weight=0)  # image rail fixed

        # ----- LEFT (wrapped title + details) -----
        title_lbl = tk.Label(
            card,
            text=book['title'],
            font=('Arial', 12, 'bold'),
            wraplength=360,
            justify='left'
        )
        title_lbl.grid(row=0, column=0, sticky='w', padx=(0, 10))

        details = (
            f"Call Number: {book['call_number']}\n"
            f"Zone {book['zone']}, Bay {book['bay']}, "
            f"Shelf {book['shelf']} ({book['side']} side)"
        )
        tk.Label(card, text=details, justify='left').grid(
            row=1, column=0, sticky='w', padx=(0, 10), pady=(4, 0)
        )

        # ----- RIGHT (image stacked over Navigate) -----
        right = tk.Frame(card)
        right.grid(row=0, column=1, rowspan=2, sticky='ne')

        # Load image if provided
        img_obj = None
        if 'image' in book and book['image']:
            img_obj = self._load_book_image(book['image'], size=(90, 120))

        if img_obj is None:
            # Simple placeholder box if missing
            ph = tk.Canvas(right, width=90, height=120, highlightthickness=0)
            ph.create_rectangle(1, 1, 89, 119)
            ph.create_text(45, 60, text="No\nImage", justify='center')
            ph.pack(anchor='e', pady=(0, 6))
        else:
            img_lbl = tk.Label(right, image=img_obj)
            img_lbl.image = img_obj
            img_lbl.pack(anchor='e', pady=(0, 6))

            # Make the image clickable to show the full image
            img_lbl.bind("<Button-1>", lambda e, img=book['image']: self.show_full_image(img))

        # Navigate button under the image (no-op in MOCK_UI)
        btn = tk.Button(
            right,
            text='Navigate',
            command=(lambda b=book: self.start_navigation(b))
        ) 
        btn.pack(anchor='e')
        self.buttons.append(btn)

    def start_navigation(self, book):
        if self.is_navigating:
            messagebox.showinfo('Robot Busy', 'Robot is currently navigating. Please wait.')
            return
        self.current_target_book = book
        self.is_navigating = True
        self.navigation_phase = 'to_shelf'
        self.nav_phase_pub.publish('to_shelf')
        self._disable_buttons()
        coord = book['coordinates']
        self.status_label.config(
            text=(f"Your book: \"{book['title']}\"\n"
                  f"Location: Bay {book['bay']}, Shelf {book['shelf']}\nNavigating to book..."))
        self.status_label.update_idletasks()
        time.sleep(1)
        self._start_nav_section(book)
        self._log(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] [NAV {self.nav_session_id}] Sending goal to shelf…")

        self.publish_goal(**coord)

    def _disable_buttons(self):
        for btn in self.buttons:
            if btn.winfo_exists():
                btn.config(state='disabled')

    def _enable_buttons(self):
        if self.battery_percentage < BATTERY_THRESHOLD:
            return
        for btn in self.buttons:
            btn.config(state='normal')

    def publish_goal(self, x, y, z, w):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z, goal.pose.orientation.w = z, w
        self.goal_pub.publish(goal)
        self.last_goal_pub.publish(goal)
        rospy.loginfo(f"[DEBUG] Goal sent: ({x}, {y}, z={z}, w={w})")

    def play_shelf_beep(self, shelf_level):
        # Full melody
        self.sound_pub.publish(Sound(value=SOUND_ALERT))
        time.sleep(4)
        # Soft beeps
        for _ in range(min(shelf_level, 5)):
            self.sound_pub.publish(Sound(value=SOUND_BEEP))
            time.sleep(2)
        # End melody
        self.sound_pub.publish(Sound(value=SOUND_END))

    def battery_callback(self, msg):
        voltage = msg.voltage
        # map voltage to percent
        perc = int((voltage - 10.99) / (12.31 - 10.99) * 100)
        self.battery_percentage = max(0, min(perc, 100))
        self.battery_label.config(text=f"Battery: {self.battery_percentage}% ({voltage:.2f}V)")
        if self.battery_percentage < BATTERY_THRESHOLD:
            self._disable_buttons()
            self.status_label.config(text='Low battery. Please recharge.')
            
    def _pretty_detection_message(self, label: str) -> str:
        if self.current_target_book and self.navigation_phase == 'to_shelf':
            zone = self.current_target_book.get('zone', '?')
            side = (self.current_target_book.get('side', '') or '').lower()
            return f"Found {label} while navigating to Zone {zone} {side} side"
        return f"Found {label}"


    def detection_callback(self, msg):
        txt = msg.data
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        sid = f"[NAV {self.nav_session_id}] " if self.nav_session_id else ""

        # --- Show contextual message on screen; keep brackets with coords in LOG ---
        # Case A: detailed YOLO line with bbox
        m = re.match(
            r".*Detected:\s*([^\s]+)\s*\([^)]*\)\s*at\s*\[x1=([\d.]+),\s*y1=([\d.]+),\s*x2=([\d.]+),\s*y2=([\d.]+)\]",
            txt
        )
        if m:
            label, x1, y1, x2, y2 = m.groups()
            ui_text = self._pretty_detection_message(label)
            bbox_str = f"[x1={float(x1):.1f}, y1={float(y1):.1f}, x2={float(x2):.1f}, y2={float(y2):.1f}]"

            # UI (no coords)
            self.detection_label.config(text=ui_text)

            # LOG (with coords in brackets)
            self._log(f"[{timestamp}] {sid}{ui_text} {bbox_str}")
            return

        # Case B: simple detector line "Found label at (x, y)"
        m = re.match(r"Found\s+(.+?)\s+at\s+\(([-\d]+),\s*([-\d]+)\)", txt)
        if m:
            label, x, y = m.groups()
            ui_text = self._pretty_detection_message(label)
            pix_str = f"[pixel=({x}, {y})]"

            # UI (no coords)
            self.detection_label.config(text=ui_text)

            # LOG (with coords in brackets)
            self._log(f"[{timestamp}] {sid}{ui_text} {pix_str}")
            return


    def navigation_feedback_callback(self, result):
        status = result.status.status
        if status == 3:  # reached
            if self.navigation_phase == 'to_shelf' and not self.has_played_shelf_sound:
                self.has_played_shelf_sound = True
                self.status_label.config(
                    text=f"Arrived at \"{self.current_target_book['title']}\". Playing alert...")
                self.play_shelf_beep(self.current_target_book.get('shelf', 1))
                rospy.Publisher('/pause_detection', String, queue_size=1).publish('pause')
                threading.Thread(target=self.delayed_rest_navigation, daemon=True).start()
                self.status_label.config(text='Returning back to rest position.')
            elif self.navigation_phase == 'to_rest':
                self.is_navigating = False
                self.has_played_shelf_sound = False
                self.navigation_phase = 'idle'
                self._enable_buttons()
                self.nav_phase_pub.publish('idle')
                self.status_label.config(text='Ready. Robot at rest position.')
                self._end_nav_section("Reached & back to rest")

        elif status == 4:  # aborted
            self.status_label.config(text='Navigation failed. Obstacle or unreachable area.')
            self._log(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] [NAV {self.nav_session_id}] Navigation aborted.")
            self._end_nav_section("Aborted")

            self.is_navigating = False
            self._enable_buttons()

    def delayed_rest_navigation(self):
        time.sleep(1.5)
        rospy.Publisher('/pause_detection', String, queue_size=1).publish('pause')
        self.navigation_phase = 'to_rest'
        self.publish_goal(**REST_COORD)
        self.nav_phase_pub.publish('to_rest')

if __name__ == '__main__':
    TurtlebotUI()

