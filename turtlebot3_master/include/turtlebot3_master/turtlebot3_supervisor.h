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
#define _USE_MATH_DEFINES

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_master/Super.h>
#include <turtlebot3_master/ScanData.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <cmath>

 

#include <tf2/LinearMath/Quaternion.h>



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

struct Quaternion
{
    double x, y, z, w;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

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
  int currentController = 0;
  
  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher super_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher move_base_goal_pub_;
  ros::Publisher move_base_goal_cancel_pub_;
            
  // ROS Topic Subscribers
  ros::Subscriber cmd_vel2_sub_;
  ros::Subscriber cmd_vel3_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_scan_eval_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  geometry_msgs::Twist cmdVel2Msg_;
  geometry_msgs::Twist cmdVel3Msg_;

  double scan_data_[3] = {0.0, 0.0, 0.0};
    bool goalSent = false;
  int finiteStateMachineState_ = 0;

    float maxSpeed_ = 0.22;
    float slowSpeed_ = 0.1;
    float safeZoneDist_ = 0.2;
    float bufferZoneDist_ = 0.4;
    float minRangeFront_, minRangeBack_, minRange_;
	int drivingDirection = 0;

    float currentPositionX_;
    float currentPositionY_;
    float currentOrientationZ_;
    float currentOrientationW_;
	
	float chunkValues[8] = {};
	float chunkWeights[8] = {1.0, -0.1, -0.1, 0.2, 0.2, -0.1, -0.1, 1.0};
	float collectedChunk_ = 0.0;

	float chunkValuesTurnedLeft[8] = {};
	float collectedChunkTurnedLeft_ = 0.0;

	float chunkValuesTurnedRight[8] = {};
	float collectedChunkTurnedRight_ = 0.0;

	// 1=SafeMode 2=SemiAutonomy 3=FullAutonomy
	int testCondition = 1;

    bool circlePressed_ = false;

  float scanData[360][3] = {0};


  double tb3_pose_;
  double prev_tb3_pose_;


  // Function prototypes
  float findMinDist(float startLeftAngle, float angle);
  void calculateChunks(float startAngle);
  Quaternion eulerToQuat(double yaw, double pitch, double roll);
  EulerAngles ToEulerAngles(Quaternion q);
  void updateSuperCommand(float linearVelLimit, float angularVelLimit);
  void cmdVel2MsgCallBack(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdVel3MsgCallBack(const geometry_msgs::Twist::ConstPtr &msg);
  void laserScanEvalMsgCallBack(const turtlebot3_master::ScanData::ConstPtr &msg);
  void joyMsgCallBack(const sensor_msgs::Joy::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  //void odomMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};
#endif // TURTLEBOT3_SUPERVISOR_H_
