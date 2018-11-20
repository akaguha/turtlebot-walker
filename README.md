# turtlebot-walker
[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
Implementation of a simple walker algorithm much like a Roomba robot. The robot moves forward until it reaches an obstacle, then rotates in place until the way ahead is clear, then moves forward again and repeat.

# License
```
BSD 3-Clause License

Copyright (c) 2018, Akash Guha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

# Dependencies
Following dependencies need to be installed before running the above package
- Ubuntu 16.04
- ROS Kinetic
- Gazebo version 7.0.0 or above
- Turtlebot packages
Run the following command to install turtlebot packages
```
sudo apt-get install ros-kinetic-turtlebot-*
```

# Build Instructions
Create and build a catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Source your new setup.*sh file
```
source devel/setup.bash
```
Clone the package in the src folder and build
```
cd src/
git clone --recursive https://github.com/akaguha/turtlebot-walker.git
cd ..
catkin_make
```

# Demo Instructions
To run the simulation
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch turtlebot-walker Walker.launch
```
This was without rosbag recording
To run the simulation along with recording, add the below parameter
```
roslaunch turtlebot-walker Walker.launch record_bag:=true
```
To replay the rosbag file
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/corridor.world
```
In a different terminal run
```
cd <path to catkin_ws>/src/turtlebot-walker/results
rosbag play allTopics.bag
```
You can see the turtle bot moving the gazebo environment with the recorded messages
To monitor the recorded messages and topics run the following command
```
cd <path to catkin_ws>/src/turtlebot-walker/results
rosbag info allTopics.bag
```
To terminate press CTRL+C

