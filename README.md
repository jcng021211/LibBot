# **LibBot – Smart Library Assistant System (TurtleBot3 Burger)**  
AI-Powered Navigation & Object Detection for UTeM Library

Demo Video:  
**https://drive.google.com/drive/folders/1eP3tycHQVQlP2pS14ALkA_i3lBY_dzbg?usp=drive_link**

---

## 📦 **Dataset**
LibBot Object Detection Model (YOLOv8):  
🔗 **https://www.kaggle.com/models/ngjuncherng/libbot-object-detection**

---

## 🚀 **Installation & Setup**

### **1. Prepare Hardware**
- Fully charge the **TurtleBot3 Burger**.
- Place the robot at the **designated start location**  
  (table area, 4th floor library zone).

### **2. Connect to TurtleBot**
On Ubuntu laptop:

1. Ensure laptop WiFi is the **same network** as TurtleBot.
2. Open terminal and connect via SSH:
   ```bash
   ssh ubuntu@<turtlebot_ip_address>

### **3. Launch Robot Startup**
After SSH login:
```bash
roslaunch <package_name> turtlebot_startup.launch
```

### **4. Launch Laptop-Side Nodes**
Open another terminal on the laptop:
```bash
roslaunch <package_name> laptop_startup.launch
```

---

## 📘 **Using LibBot**
1. Open the LibBot GUI.
2. Enter book title or call number in the search bar.
3. Select a result from the automatically filtered list.
4. Click Navigate, and LibBot will guide you to the correct bookshelf zone.

---

## 🛠 **Maintenance Guidelines**
🔋 **Battery**
- Replace or charge the battery when the GUI shows Low Battery.
- Full charge time: ~2 hours
- Operating time per charge: 40–45 minutes

📚 **Data & Model Updates**
- Update book_locations.json when adding new books.
- Retrain or update YOLOv8 models if:
  - New book covers are added
  - New rubbish types need detection

🗺 **Mapping**
If the library layout changes:
- Regenerate the SLAM map using slam_toolbox in mapping mode.
- Save the new map for navigation.

---

## 🔐 **Safety Instructions**
- Operate only in supervised environments.
- Avoid risky locations such as stairs and escalators.
- Keep walkways clear of liquids, cables, or obstacles.
- Do not physically force the robot while it is moving.
- Do not stand directly in front of the robot to avoid false detections.

---

## 🤖 **About LibBot**
LibBot is an autonomous library assistant robot developed using:
- TurtleBot3 Burger
- ROS Noetic
- YOLOv8 Object Detection
- SLAM Toolbox
- Python Tkinter GUI
The system helps students locate books, identifies misplaced books and rubbish, and navigates safely within a library testbed environment.

---

## 👤 **Developer**
Ng Jun Cherng

UTeM Final Year Project – Smart Library Assistant System
