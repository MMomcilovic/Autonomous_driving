# Autonomous_driving

This is repo for implementation of Autonomouse driving for Soft Computing project.

This repo contains module for **Traffic sign detection** using YOLOv8 and **Lane detection** using openCV.

DEMO

![1686321066560](image/README/1686321066560.png)

To run this code you need to config [this](https://github.com/ECC-BFMC/Simulator) Gazebo Simulator on Linux. On this repo you have documentation on how to setup the simulator and how to run it.

Essentially what you need is **Ubuntu 20.04, ROS 1** and **Python 3.7+**. Create new ROS package or add files to existing one in simulator.

#1 Open terminal and launch the simulation

```bash
source devel/setup.bash
roslaunch sim_pkg map_with_all_objects.launch
```

#2 Open 2 more terminals and run line_detection and traffic_sign_detection nodes

```bash
source devel/setup.bash
rosrun <package_name> line_detection.py
```

```bash
source devel/setup.bash
rosrun <package_name> traffic_sign_detection.py
```

#3 Finally open one more terminal and run the brain node, car should start moving.

```bash
source devel/setup.bash
rosrun <package_name> brain.py
```

If you want to train or fine-tune model for traffic sign detection you need [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics).
