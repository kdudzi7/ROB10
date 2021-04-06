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

#include "turtlebot3_master/turtlebot3_automation.h"

Turtlebot3Automation::Turtlebot3Automation()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Automation::~Turtlebot3Automation()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Automation::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7; //orig 0.7
  check_side_dist_    = 0.6; // orig 0.6

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel3", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Automation::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Automation::odomMsgCallBack, this);
  super_sub_ = nh_.subscribe("super_cmd", 10, &Turtlebot3Automation::superCallBack, this);

  return true;
}

void Turtlebot3Automation::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Automation::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Automation::superCallBack(const turtlebot3_master::Super::ConstPtr &msg)
{
  linearVelLimit = msg->linearVelLimit;
  angularVelLimit = msg->angularVelLimit;
}

void Turtlebot3Automation::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  if(abs(linear) > linearVelLimit)
  {
    linear = linearVelLimit;
  }
  if(abs(angular) > angularVelLimit)
  {
    angular = angularVelLimit;
  }

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Automation::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;


	  switch(turtlebot3_state_num)
	  {
	    case GET_TB3_DIRECTION:
	      if (scan_data_[CENTER] > check_forward_dist_)
	      {
		if (scan_data_[LEFT] < check_side_dist_)
		{
		  prev_tb3_pose_ = tb3_pose_;
		  turtlebot3_state_num = TB3_RIGHT_TURN;
		}
		else if (scan_data_[RIGHT] < check_side_dist_)
		{
		  prev_tb3_pose_ = tb3_pose_;
		  turtlebot3_state_num = TB3_LEFT_TURN;
		}
		else
		{
		  turtlebot3_state_num = TB3_DRIVE_FORWARD;
		}
	      }

	      if (scan_data_[CENTER] < check_forward_dist_)
	      {
		prev_tb3_pose_ = tb3_pose_;
		turtlebot3_state_num = TB3_RIGHT_TURN;
	      }
	      break;

	    case TB3_DRIVE_FORWARD:
	      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
	      turtlebot3_state_num = GET_TB3_DIRECTION;
	      break;

	    case TB3_RIGHT_TURN:
	      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
		turtlebot3_state_num = GET_TB3_DIRECTION;
	      else
		updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
	      break;

	    case TB3_LEFT_TURN:
	      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
		turtlebot3_state_num = GET_TB3_DIRECTION;
	      else
		updatecommandVelocity(0.0, ANGULAR_VELOCITY);
	      break;

	    default:
	      turtlebot3_state_num = GET_TB3_DIRECTION;
	      break;
	  }

  

		
  
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_automation");
  Turtlebot3Automation turtlebot3_automation;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_automation.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
