1. fsd_pc.launch.py
    - extrinsic
    - lane_detect
    - lane_control

2. fsd_bot.launch.py
    - ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
    - ros2 launch aruco_yolo aruco_yolo.launch.py
    - ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
    - ros2 run turtlebot_moveit turtlebot_arm_controller




ros2 launch turtlebot3_manipulation_bringup hardware.launch.py