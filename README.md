# RobotTeknik
ROS packages used for laboratory exercises in the Intorduction to Robotics course at Orebro University

# Dependencies
Packages here are tested for ROS2 Humble. All ros dependnecies should be specified in individual packages. Base dependency is ros2_control.

# Building
Just get the dependencies with
```
rosdep update
cd $ROS_HOME
rosdep install --from-paths src -y --ignore-src
```
Followed by build:
```
cd .. && colcon build
```

# Testing your build
Try running:
```
ros2 launch rrbot rrbot.launch.py
```

