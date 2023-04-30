# autonomous_opencv
simple navigation with obstacle avoidance with 2d lidar  and odom opencv is used for visualization. 
This package is not for practicle application its more like about learning mobile robot navigation.
## Deps
1) Opencv 4.2
2) ros2 humble
3) turtlebot3_gazebo  

## Build 
```
cd autonomous_opencv
mkdir build 
cd build 
cmake ..
make 
./autonomous_opencv
```

## Run
for now you can test it with gazebo simulation

```

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

in next terminal 
```
./autonomous_opencv
```