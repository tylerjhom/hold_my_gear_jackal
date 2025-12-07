# Hold My Gear – Jackal Firefighter Following Robot

Follow-me Clearpath Jackal robot that carries gear for Austin Fire Department’s Air Shop crew.

The robot uses an Intel RealSense depth camera and YOLOv11 for person detection, with two main operating modes:

1. **Nav2-based follower** – YOLO publishes waypoints to Nav2, which handles mapping, path planning, and obstacle avoidance.
2. **Direct velocity follower** – YOLO computes a relative target and publishes `/cmd_vel` directly (no obstacle avoidance).

This project was developed for **ME396P – Application Programming for Engineers** at UT Austin.

---

## 1. Repository Layout

```text
hold-my-gear-jackal/
├── README.md
├── requirements.txt
├── docs/
│   ├── 02_robot_setup.md
│   └── 03_offboard_setup.md
├── robot/
│   ├── config/
│   │   └── robot.yaml
│   └── jackal_yolo_follow/
│       ├── package.xml
│       ├── setup.py, setup.cfg
│       ├── resource/
│       ├── jackal_yolo_follow/
│       │   ├── yolo_nav2_follower*.py
│       │   ├── nav_to_pose_test*.py
│       │   ├── yolo_follower.py
│       │   └── ...
│       ├── launch/
│       │   ├── yolo_nav2_follow.launch.py
│       │   └── yolo_direct_follow.launch.py
│       └── experimental/
└── offboard/
    ├── rviz/
    │   └── jackal_follow.rviz
    ├── config/
    │   └── cyclonedds.xml
    └── scripts/
        └── start_offboard_viz.sh
