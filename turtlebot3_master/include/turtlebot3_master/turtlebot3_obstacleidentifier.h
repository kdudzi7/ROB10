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

#ifndef TURTLEBOT3_OBSTACLEIDENTIFIER_H_
#define TURTLEBOT3_OBSTACLEIDENTIFIER_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <turtlebot3_master/ScanData.h>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2


class Turtlebot3ObstacleIdentifier
{
 public:
  Turtlebot3ObstacleIdentifier();
  ~Turtlebot3ObstacleIdentifier();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;


  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher scan_eval_pub_;
            
  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_scan_eval_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  float scan_data_[360][3] = {0.0};



  double tb3_pose_;
  double prev_tb3_pose_;

  // Function prototypes
  void updateScanEvalMsg();
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
};
#endif // TURTLEBOT3_OBSTACLEIDENTIFIER_H_
