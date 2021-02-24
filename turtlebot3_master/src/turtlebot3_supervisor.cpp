
/*******************************************************************************
* 
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "turtlebot3_master/turtlebot3_supervisor.h"

Turtlebot3Supervisor::Turtlebot3Supervisor()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node

  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Supervisor::~Turtlebot3Supervisor()
{
  updatecommandVelocity(0.0, 0.0);
  nh_.deleteParam("linear_max_vel");	
  nh_.deleteParam("angular_max_vel");	
  nh_.deleteParam("controller");
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Supervisor::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");
  nh_.setParam("linear_max_vel", 0.1);
  nh_.setParam("angular_max_vel", 0.1);
  nh_.setParam("controller", 0);

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7; //orig 0.7
  check_side_dist_    = 0.6; // orig 0.6

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Supervisor::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Supervisor::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Supervisor::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  if(msg->pose.pose.position.x > 0)
  {
	nh_.setParam("linear_max_vel", 0.5);
  }
  else
  {
	nh_.setParam("linear_max_vel", 0.1);
  }
}

void Turtlebot3Supervisor::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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

void Turtlebot3Supervisor::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Supervisor::controlLoop()
{
  
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_supervisor");
  Turtlebot3Supervisor turtlebot3_supervisor;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_supervisor.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
