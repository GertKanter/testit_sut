TestIt: a Scalable Long-Term Autonomy Testing Toolkit for ROS - SUT additions
=============================================================================

Toolkit webpage at is at [wiki.ros.org/testit](http://wiki.ros.org/testit)

This package adds the necessary services that needs to be executed in the SUT (System Under Test) and must be added to the robot software stack which is running the ROS master (`roscore`). Currently, this is required for code coverage analysis.

# Installation

## Get the package

Clone the repository to your workspace (e.g., `~/catkin_ws`).

```
cd ~/catkin_ws/src
git clone https://github.com/GertKanter/testit_sut
cd ..
catkin_make
```
## Add to stack launch

Add the package to your stack launch file.
