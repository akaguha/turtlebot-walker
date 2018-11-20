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
*  @file    Walker.hpp
*  @author  Akash Guha(akaguha@terpmail.umd.edu)
*  @version 1.0 
*
*  @brief ENPM808X, Programming Assignment - Working with Gazebo
*
*  @section DESCRIPTION
*
*  This program is the header implementation of the Walker class
*
*/
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


/**
 *  @brief Class Walker
 *
 *  This class scans the laser data from the turtlebot,computes distance from
 *  obstacles and publishes velocities accordingly to the turtlebot to
 *  demonstrate a walker algorithm
 */
class Walker {
  private:
  	geometry_msgs::Twist msg;
  	ros::NodeHandle nh;  //  Nodehandle object
  	ros::Publisher pubVelocity;  //  Publisher object
  	ros::Subscriber subScanner;  //  Subcriber object
  	float minDist=0.7;  //  Stores the threshold distance from obstacle
  	float scanData;  //  Stores data from turtlebot's laser scanner
  	float dist;
  public:
/**
*   @brief  Constructor for Walker class
*
*   @param  nh as nodehandle object
*
*   @return void
*/
  Walker(ros::NodeHandle &nh);
/**
*   @brief  Destructor for Walker class
*
*   @param  none
*
*   @return void
*/
  ~Walker();
/**
*   @brief  Function to publish linear velocities to turtlebot
*
*   @param  none
*
*   @return void
*/
  void moveForward();
/**
*   @brief  Function to publish angular velocities to turtlebot
*
*   @param  none
*
*   @return void
*/
  void turn();
/**
*   @brief  Function to determine the presence of an obstacle
*
*   @param  none
*
*   @return true or false based on distance of the turtlebot from obstacle
*/
bool obstacleDetected();
/**
*   @brief  Laser scanner callback function
*
*   @param  readings as pointer to the laser scanner data received from turtlebot
*
*   @return void
*/
  void scannerCallback(const sensor_msgs::LaserScan::ConstPtr& readings);

};

#endif // INCLUDE_WALKER_HPP_
