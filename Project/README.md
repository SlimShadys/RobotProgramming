# RobotProgramming - 2D Multi-Robot Simulator
### Project by Gianmarco Scarano | ID: 2047315

<p align="center">
  <img title="Demo" src="media/Demo.gif" align="center" width="700">
</p>

This repo contains the project assigned during the Robot Programming course at AI & Robotics Master's Degree at University of Sapienza in Rome, Italy.<br>

The task is to extend the 2D Robot simulator illustrated during the course by implementing some non-trivial modifications.
## Todo List
❌ = Not yet implemented <br>
⚠️ = Work in progress<br>
✅ = Done<br>

- Implementing **ROS** Support | ⚠️ (90%)
- Implementing a configuration system by file | ✅
- Complete the CMakeLists provided by us | ✅

## Run the code
To launch this simulator, run its node as follows:
```sh
rosrun mrsim mrsim_node <config>.json
```
Various configs can be found inside "workspace/src/mrsim/test_data/" directory.