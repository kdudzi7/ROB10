
/*******************************************************************************
* 
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "turtlebot3_master/turtlebot3_obstacleidentifier.h"

Turtlebot3ObstacleIdentifier::Turtlebot3ObstacleIdentifier()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node

  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3ObstacleIdentifier::~Turtlebot3ObstacleIdentifier()
{
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3ObstacleIdentifier::init()
{
  // initialize ROS parameter

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7; //orig 0.7
  check_side_dist_    = 0.6; // orig 0.6



  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;


  // initialize publishers
  scan_eval_pub_ = nh_.advertise<turtlebot3_master::ScanData>("scan_eval", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3ObstacleIdentifier::laserScanMsgCallBack, this);

  return true;
}


void Turtlebot3ObstacleIdentifier::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[360] = {0, 30, 330};


  for (int i = 0; i < 360; i++)
  {
      scan_data_[i][0] = (msg->angle_min + msg->angle_increment*i) * RAD2DEG;

    if (std::isinf(msg->ranges.at(i)))
    {
      scan_data_[i][1] = msg->range_max;
      scan_data_[i][2] = 0;
    }
    else
    {
      scan_data_[i][1] = msg->ranges.at(i);
      scan_data_[i][2] = 1;
    }
  }

  updateScanEvalMsg();
}

void Turtlebot3ObstacleIdentifier::updateScanEvalMsg()
{

  turtlebot3_master::ScanData scanData_msg;


  
  for(int i = 0; i < (sizeof(scan_data_)/sizeof(*scan_data_)); i++)
  {
    scanData_msg.angles.push_back(scan_data_[i][0]);
    scanData_msg.ranges.push_back(scan_data_[i][1]);
    scanData_msg.type.push_back(scan_data_[i][2]);
  }





  scan_eval_pub_.publish(scanData_msg);
}



/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3ObstacleIdentifier::controlLoop()
{
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_obstacleIdentifier");
  Turtlebot3ObstacleIdentifier turtlebot3_obstacleIdentifier;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_obstacleIdentifier.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
