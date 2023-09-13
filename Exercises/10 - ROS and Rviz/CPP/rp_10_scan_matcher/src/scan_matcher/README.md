# README
## Setup ROS
For every terminal you use, remember to first source ROS:

```sh
source /opt/ros/noetic/setup.bash
```

Open a new terminal and after sourcing ROS, statup the roscore:

```sh
roscore
```

## Decompress Rosbag file

To decompress the bag:
```sh
rosbag decompress data/kuka_short.bag
```

This should produce two bags:
- kuka_short.bag [Decompressed]
- kuka_short.orig.bag [Compressed]

Remember to use only the decompressed one.

## Compilation
To compile your node, create your workspace folder. Its a good practise to have different workspaces for different projects:
```sh
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir robot_programming
cd robot_programming
mkdir src
cd src
catkin_init_workspace
cd ..
```

Now create a soft link from your exercise folder to `catkin_ws/robot_programming/src` folder:
```sh
ln -s /home/$USER/<path to exercise folder> /home/$USER/catkin_ws/robot_programming/src/rp_10_scan_matcher
```

Finally you can compile your package using _catkin_:

```sh
catkin build
```

If no errors occurred, you can source the workspace

```sh
source devel/setup.bash
```

## Execution 
To run your code, use _rosrun_ and **REMEMBER** to also remap the node's input topic with the `~scan:=TOPIC_NAME`.
In the context of this exercise, if you use the `kuka_short.bag`:
```
rosrun scan_matcher scan_matcher_node ~scan:=/robot_0/base_scan
```

## RViz 
To run RViz, open a new terminal and go to the `rp_10_scan_matcher` folder, then type:

```sh
rviz -d config.rviz
```


