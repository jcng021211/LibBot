📚 LibBot — Smart Library Assistant System (TurtleBot3 Burger)

An autonomous library assistant robot designed to guide users to bookshelves, detect misplaced books, and identify rubbish using ROS Noetic, YOLOv8, SLAM, and a custom Tkinter GUI interface.

🚀 Features
🔍 Book Search & Navigation

Search by book title or call number

Filtered book results shown in interactive GUI cards

Navigation to the correct bookshelf zone using move_base

Automatic re-planning when obstacles are detected

🤖 Visual Detection Module

YOLOv8-based object detection supporting:

10 specific UTeM book covers

Unified “rubbish” class (battery, cardboard, paper, plastic, etc.)

Confidence filtering (≥ 60%)

False-positive reduction using:

IOU filtering

5–10 second cooldown periods

Reflection and noise suppression logic

🧠 Autonomous Navigation Behaviour

Dynamic obstacle avoidance

Reverse-turn avoidance manoeuvre

Goal re-sending and recovery behaviour

Navigation phases: idle → to_shelf → to_rest

🖥️ User Interface (Tkinter GUI)

Book search & navigation panel

Robot status display (navigation, avoidance, pause)

Battery level indicator with low-battery lockout

Real-time object detection notifications

📁 Project Repository Structure
LibBot/
│
├── Smart_Library.py          # Main GUI application
├── ros_yolo_detection.py     # YOLOv8 ROS detection node
├── book_locations.json       # Predefined coordinates for each book title
├── laptop_startup.launch     # Launch file for laptop ROS nodes
├── turtlebot_startup.launch  # Launch file for robot ROS nodes
│
├── /models                   # YOLOv8 weight files (not uploaded)
├── /maps                     # SLAM map files
└── /docs                     # Final report, diagrams, documentation

📦 Dataset & Model
🧱 Object Detection Models (Books & Rubbish)

Kaggle Model Link:
🔗 https://www.kaggle.com/models/ngjuncherng/libbot-object-detection

Models include:

10 specific UTeM library book covers

1 unified class: rubbish

Trained using YOLOv8s on 10,000+ mixed real + synthetic images

🛠️ Installation & Setup
1️⃣ Hardware Requirements

TurtleBot3 Burger

Fully charged battery (40–45 min runtime)

Ubuntu 20.04 + ROS Noetic laptop

Shared WiFi network (TurtleBot + Laptop must be on the same network)

2️⃣ Connect to TurtleBot

SSH into robot:

ssh ubuntu@<turtlebot_ip>


Start robot-side ROS nodes:

roslaunch libbot turtlebot_startup.launch


Start laptop-side ROS nodes:

roslaunch libbot laptop_startup.launch

🧭 Using LibBot

Run Smart_Library.py

Enter a book title or call number

Select a result card

Click Navigate

The robot will autonomously move to the correct bookshelf zone

🧹 Maintenance Guidelines

Recharge or replace battery when indicator shows Low Battery

Full charge time: ~2 hours

Update:

book_locations.json when new books are added

YOLO models when adding new book covers or rubbish types

If library layout changes:

Remap using slam_toolbox and update /maps

⚠️ Safety Instructions

Operate only in supervised environments

Avoid stairs, escalators, and wet floors

Keep walkways clear during testing

Do not block or force the robot

Avoid standing directly in front of the robot (prevents detection confusion)

👨‍💻 Developer

Ng Jun Cherng
Smart Library Assistant System (FYP 2024/2025)
ROS Noetic · YOLOv8 · TurtleBot3 Burger · Python · Tkinter · SLAM
