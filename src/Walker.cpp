/**
*  BSD 3-Clause License
*
*  Copyright (c) 2018, Akash Guha
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*3. Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*  @file    Walker.cpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Programming Assignment - Working with Gazebo
*
*  @section DESCRIPTION
*
*  This file is the Walker class's cpp file which contains the body of all the
*  methods defined in the Walker class
*
*/
#include "Walker.hpp"  //  Walker class header
/**
*   @brief  Constructor for Walker class
*
*   @param  nh as nodehandle object
*
*   @return void
*/
Walker::Walker(ros::NodeHandle& nh) {
	//  Setting up a publisher with a master
	pubVelocity = nh.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1000);
	//  Setting up a subscriber with a master
	subScanner = nh.subscribe <sensor_msgs::LaserScan> ("/scan", 100, &Walker::scannerCallback, this);
	ros::Rate loop_rate(5);  //  Setting the looping rate
	while (ros::ok()){
	  if (obstacleDetected()){
		  turn();  //  Calling the turn method to avoid obstacle
	  }else{
		  moveForward();  //  Calling the moving forward method in absence of an obstacle
	  }
	  //  Using publish method to send velocity values to turtlebot
	  pubVelocity.publish(msg);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
}
/**
*   @brief  Destructor for Walker class
*
*   @param  none
*
*   @return void
*/
Walker::~Walker() {
}
/**
*   @brief  Laser scanner callback function
*
*   @param  readings as pointer to the laser scanner data received from turtlebot
*
*   @return void
*/
void Walker::scannerCallback(const sensor_msgs::LaserScan::ConstPtr& readings) {
	dist = minDist;  //  Initialize the dist variable with the threshold distance value
	int sz = readings->ranges.size();  //  Stores the size of laser scan array
	for(int i=0; i<sz; i++) {
		scanData = readings->ranges[i];  //  Stores the laser scan data
	   //  Based on the received data assigning value to dist variable
       if (scanData < dist) {
       	dist = scanData;
       	ROS_WARN_STREAM("Distance " << dist << " less than the threshold value");
       }
	}
}
/**
*   @brief  Function to determine the presence of an obstacle
*
*   @param  none
*
*   @return true or false based on distance of the turtlebot from obstacle
*/
bool Walker::obstacleDetected() {
	if(dist < minDist) {
		return true;
	}else {
		return false;
	}
}
/**
*   @brief  Function to publish linear velocities to turtlebot
*
*   @param  none
*
*   @return void
*/
void Walker::moveForward() {
	ROS_INFO_STREAM("Path is clear to go forward");
  //  Setting a linear velocity and making angular velocity zero
  msg.linear.x = 0.1;
  msg.angular.z = 0.0;
}
/**
*   @brief  Function to publish angular velocities to turtlebot
*
*   @param  none
*
*   @return void
*/
void Walker::turn() {
	ROS_INFO_STREAM("Obstacle ahead, turning");
  //  Setting an angular velocity and making linear velocity zero
  msg.linear.x = 0.0;
  msg.angular.z = 0.5;
}
