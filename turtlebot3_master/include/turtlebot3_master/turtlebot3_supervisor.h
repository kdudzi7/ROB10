/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef TURTLEBOT3_SUPERVISOR_H_
#define TURTLEBOT3_SUPERVISOR_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_master/Super.h>
#include <turtlebot3_master/ScanData.h>



#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class Turtlebot3Supervisor
{
 public:
  Turtlebot3Supervisor();
  ~Turtlebot3Supervisor();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  double linMaxVel = 0.3;
  double angMaxVel = 1.5;
  int currentController = 1;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher super_pub_;
            
  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_scan_eval_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  float scanAngles[30] = {0};
  float scanRanges[30] = {0};
  float scanTypes[30] = {0};

  double tb3_pose_;
  double prev_tb3_pose_;

  // Function prototypes
  void updatecommandVelocity(int controller, float linearVelLimit, float angularVelLimit, float minDist);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void laserScanEvalMsgCallBack(const turtlebot3_master::ScanData::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // TURTLEBOT3_SUPERVISOR_H_
