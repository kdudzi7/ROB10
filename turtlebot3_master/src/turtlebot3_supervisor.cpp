
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
  updateSuperCommand(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Supervisor::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7; //orig 0.7
  check_side_dist_    = 0.6; // orig 0.6

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

    finiteStateMachineState_ = 1;

  // initialize publishers
  super_pub_ = nh_.advertise<turtlebot3_master::Super>("super_cmd", 10);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    move_base_goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
    move_base_goal_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);

  // initialize subscribers
  cmd_vel2_sub_ = nh_.subscribe("cmd_vel2", 10, &Turtlebot3Supervisor::cmdVel2MsgCallBack, this);
  cmd_vel3_sub_ = nh_.subscribe("cmd_vel3", 10, &Turtlebot3Supervisor::cmdVel3MsgCallBack, this);
  laser_scan_eval_sub_ = nh_.subscribe("scan_eval", 10, &Turtlebot3Supervisor::laserScanEvalMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Supervisor::odomMsgCallBack, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Turtlebot3Supervisor::joyMsgCallBack, this);

  return true;
}

void Turtlebot3Supervisor::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    currentPositionX_ = msg->pose.pose.position.x;
    currentPositionY_ = msg->pose.pose.position.y;
    currentOrientationZ_ = msg->pose.pose.orientation.z;
    currentOrientationW_ = msg->pose.pose.orientation.w;
  if(msg->pose.pose.position.x > 0)
  {
    //updateSuperCommand(2,0.22,2.0,1.0);
  }
  else
  {
    //updateSuperCommand(3,0.22,2.0,1.0);
  }
}

void Turtlebot3Supervisor::cmdVel2MsgCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmdVel2Msg_.linear.x = msg->linear.x;
    cmdVel2Msg_.angular.z = msg->angular.z;
	if(cmdVel2Msg_.linear.x < 0.0)
	{
		drivingDirection = -1;
	}
	else if(cmdVel2Msg_.linear.x > 0.0)
	{
		drivingDirection = 1;
	}
}

void Turtlebot3Supervisor::cmdVel3MsgCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmdVel3Msg_.linear.x = msg->linear.x;
    cmdVel3Msg_.angular.z = msg->angular.z;
}


void Turtlebot3Supervisor::laserScanEvalMsgCallBack(const turtlebot3_master::ScanData::ConstPtr &msg)
{
    for(int i = 0; i < 360; i++)
    {
        scanData[i][0] = msg->angles[i];
        scanData[i][1] = msg->ranges[i];
        scanData[i][2] = msg->type[i];
    }
}

void Turtlebot3Supervisor::joyMsgCallBack(const sensor_msgs::Joy::ConstPtr &msg)
{
        //ROS_WARN("button %i", msg->buttons[1]);
    if(msg->buttons[1] == 1)
    {
        finiteStateMachineState_ = 6;
    }
    else if(msg->buttons[2] == 1)
    {
        finiteStateMachineState_ = 7;    
    }
    else
    {
        //finiteStateMachineState_ = 1;
    }
}

void Turtlebot3Supervisor::updateSuperCommand(float linearVelLimit, float angularVelLimit)
{
  turtlebot3_master::Super super_cmd;

  super_cmd.linearVelLimit = linearVelLimit;
  super_cmd.angularVelLimit = angularVelLimit;

  super_pub_.publish(super_cmd);
}

float Turtlebot3Supervisor::findMinDist(float startLeftAngle, float angle)
{
    float minRange = 5.0;
    if(startLeftAngle >= 360.0 || angle > 360.0 || startLeftAngle < 0.0 || angle <= 0.0)
    {
        return 666.0;
    }

    if((startLeftAngle - angle) > 0)
    {
        for(int i = (startLeftAngle - angle); i < startLeftAngle; i++)
        {
            if(scanData[i][0] < startLeftAngle && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }
           // ROS_INFO("i %f", scanData[i][0]);
        }
    }
    else
    {
        for(int i = 0; i < startLeftAngle; i++)
        {
            if(scanData[i][0] < startLeftAngle && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }
           // ROS_INFO("i %f", scanData[i][0]);
        }
        
        for(int i = (360 + startLeftAngle - angle); i < 360; i++)
        {
            if(scanData[i][0] > (360 + startLeftAngle - angle) && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }  
           // ROS_INFO("i %f", scanData[i][0]);
        }
    }

    //ROS_INFO("SmallestRange between %f and %f clockwise if %f", startLeftAngle, angle, minRange);



    return minRange;
}

void Turtlebot3Supervisor::calculateChunks(float startAngle)
{
	collectedChunk_ = 0.0;
    for(int x = 0; x < 8; x++)
    {
		if(drivingDirection == 1)
		{
			chunkWeights[0] = 1.0;
			chunkWeights[7] = 1.0;
			chunkWeights[3] = 0.2;
			chunkWeights[4] = 0.2;
		}
		else
		{
			chunkWeights[0] = 0.2;
			chunkWeights[7] = 0.2;
			chunkWeights[3] = 1.0;
			chunkWeights[4] = 1.0;
		}
		chunkValues[x] = 0.0;
        for(int i = 45 * x; i < 45 * (x+1); i++)
        {
			if(scanData[i][1] < 0.4)
			{            
				chunkValues[x] = chunkValues[x] + exp(exp(0.4 - scanData[i][1]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValues[x] = chunkValues[x] * chunkWeights[x];
		ROS_INFO("Sum between %i and %i deg = %f", (45*x), (45*(x+1)), chunkValues[x]);
			collectedChunk_ = collectedChunk_ + chunkValues[x];
    }
	ROS_INFO("CHUNK SUM IS %f", collectedChunk_);
	



	float scanDataTurnedRight[360];
	for(int i = 10; i < 360; i++)
	{
		scanDataTurnedRight[i] = scanData[i-10][1];
	}
	for(int j = 0; j < 10; j++)
	{
		scanDataTurnedRight[j] = scanData[360-10+j][1];
	}
	collectedChunkTurnedRight_ = 0.0;
    for(int x = 0; x < 8; x++)
    {
		chunkValuesTurnedRight[x] = 0.0;
        for(int i = 45 * x; i < 45 * (x+1); i++)
        {
			if(scanData[i][1] < 0.4)
			{            
				chunkValuesTurnedRight[x] = chunkValuesTurnedRight[x] + exp(exp(0.4 - scanDataTurnedRight[i]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValuesTurnedRight[x] = chunkValuesTurnedRight[x] * chunkWeights[x];

			collectedChunkTurnedRight_ = collectedChunkTurnedRight_ + chunkValuesTurnedRight[x];
    }
	ROS_INFO("CHUNK SUM TURNED RIGHT IS %f", collectedChunkTurnedRight_);
	


	float scanDataTurnedLeft[360];
	for(int i = 0; i < 350; i++)
	{
		scanDataTurnedLeft[i] = scanData[i+10][1];
	}
	for(int j = 350; j < 360; j++)
	{
		scanDataTurnedLeft[j] = scanData[0+(j-350)][1];
	}
	collectedChunkTurnedLeft_ = 0.0;
    for(int x = 0; x < 8; x++)
    {
		chunkValuesTurnedLeft[x] = 0.0;
        for(int i = 45 * x; i < 45 * (x+1); i++)
        {
			if(scanData[i][1] < 0.4)
			{            
				chunkValuesTurnedLeft[x] = chunkValuesTurnedLeft[x] + exp(exp(0.4 - scanDataTurnedLeft[i]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValuesTurnedLeft[x] = chunkValuesTurnedLeft[x] * chunkWeights[x];

			collectedChunkTurnedLeft_ = collectedChunkTurnedLeft_ + chunkValuesTurnedLeft[x];
    }
	ROS_INFO("CHUNK SUM TURNED LEFT IS %f", collectedChunkTurnedLeft_);

	


}

Quaternion Turtlebot3Supervisor::eulerToQuat(double yaw, double pitch, double roll)
{

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
           
    return q;
}

EulerAngles Turtlebot3Supervisor::ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Supervisor::controlLoop()
{
    minRangeFront_ = findMinDist(45.0, 90.0);
    minRangeBack_ = findMinDist(135.0, 90.0);
	calculateChunks(0.0);
	float angularVelocity;
	float linearVelocity;
    float bufferSpeed; 

	ROS_INFO("Driving direction %i", drivingDirection);

    switch(finiteStateMachineState_)
    {

		case 9:
            updateSuperCommand(0.22, 2.84);
            ROS_INFO("SafeZone zone, FSM is %i", finiteStateMachineState_);

			angularVelocity = abs(collectedChunk_ / 500) * 2.84;
            bufferSpeed = slowSpeed_;
            updateSuperCommand(slowSpeed_, 1.0);

			if(collectedChunkTurnedLeft_ < collectedChunk_ && collectedChunkTurnedLeft_ < collectedChunkTurnedRight_)
			{
           		cmdVel3Msg_.linear.x = cmdVel2Msg_.linear.x;//drivingDirection* slowSpeed_;
			    cmdVel3Msg_.angular.z = angularVelocity;
				ROS_WARN("Turning left");
			}
			else if(collectedChunkTurnedRight_ < collectedChunk_ && collectedChunkTurnedRight_ < collectedChunkTurnedLeft_)
			{
           		cmdVel3Msg_.linear.x = cmdVel2Msg_.linear.x;//drivingDirection * slowSpeed_;
			    cmdVel3Msg_.angular.z = -angularVelocity;
				ROS_WARN("Turning right");
			}
			else if(collectedChunk_ < collectedChunkTurnedRight_ && collectedChunk_ < collectedChunkTurnedLeft_)
			{
           		cmdVel3Msg_.linear.x = 0.0;//cmdVel2Msg_.linear.x;//drivingDirection * slowSpeed_;
			    cmdVel3Msg_.angular.z = 0.0;
				ROS_WARN("Keeping Straight");
			}

            cmd_vel_pub_.publish(cmdVel3Msg_); 
			if(collectedChunk_ < 1.0)
			{
                finiteStateMachineState_ = 9;
			}
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 9;                
            }
			else
			{
                finiteStateMachineState_ = 9;				
			}
			break;

        case 1:
            updateSuperCommand(0.22, 2.84);
            goalSent = false;
            cmd_vel_pub_.publish(cmdVel2Msg_);


            ROS_INFO("Free Drive zone, FSM is %i", finiteStateMachineState_);

        //Quaternion q;       
        //q = eulerToQuat(1.575,0.0,0.0);
        /*ROS_WARN("q.x %f", q.x);
        ROS_WARN("q.y %f", q.y);
        ROS_WARN("q.z %f", q.z);
        ROS_WARN("q.w %f", q.w);

        EulerAngles e;
        e = ToEulerAngles(q);
        ROS_WARN("e.roll %f", e.roll);
        ROS_WARN("e.pitch %f", e.pitch);
        ROS_WARN("e.yaw %f", e.yaw);
*/

			if(collectedChunk_ < 1.0)
			{
                finiteStateMachineState_ = 9;
			}
            if(minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 9;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 9;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }  
        break;
        case 2:
            {                
                float speed1 = std::min(minRangeBack_,minRangeFront_);
                bufferSpeed = (((speed1 - safeZoneDist_) / (bufferZoneDist_ - safeZoneDist_)) * (maxSpeed_ - slowSpeed_) + slowSpeed_);
                updateSuperCommand(bufferSpeed, 1.0);
                cmd_vel_pub_.publish(cmdVel2Msg_);
                ROS_INFO("Buffer zone, FSM is %i", finiteStateMachineState_);
            }

            if(minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 9;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 9;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 1;
            } 
        break;
        case 3:
             cmdVel3Msg_.linear.x = -0.02;
             cmdVel3Msg_.angular.z = 0.0;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
             ROS_INFO("Safe zone front, FSM is %i", finiteStateMachineState_);

            if(minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 4;
            }  
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 5;
            }                 
        break;
        case 4:
             cmdVel3Msg_.linear.x = 0.02;
             cmdVel3Msg_.angular.z = 0.0;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
             ROS_INFO("Safe zone back, FSM is %i", finiteStateMachineState_);

            if(minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 5;
            }                   
        break;
        case 5:
            cmdVel3Msg_.linear.x = 0.0;
            cmdVel3Msg_.angular.z = 0.5;
            cmd_vel_pub_.publish(cmdVel3Msg_);            
            ROS_INFO("Safe zone front and back, FSM is %i", finiteStateMachineState_);

            if(minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && (minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_))
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 5;
            }             
        break;
        case 6:
            if(goalSent == false)
            {

                Quaternion q = {0.0, 0.0, currentOrientationZ_, currentOrientationW_};  
                ROS_WARN("bbb %f", q.z);
                ROS_WARN("bbb %f", q.w);
                EulerAngles e;
                e = ToEulerAngles(q);
                ROS_WARN("aaa %f", e.yaw);
                float newPositionX = currentPositionX_ + 1.0 * cos(e.yaw);
                float newPositionY = currentPositionY_ + 1.0 * sin(e.yaw);
                ROS_WARN("newXPos %f", newPositionX);
                ROS_WARN("newYPos %f", newPositionY);
                move_base_msgs::MoveBaseActionGoal goalMsg;
                goalMsg.goal.target_pose.header.frame_id = "map";
                goalMsg.goal.target_pose.pose.position.x = newPositionX;
                goalMsg.goal.target_pose.pose.position.y = newPositionY;
                goalMsg.goal.target_pose.pose.position.z = 0.0;
                goalMsg.goal.target_pose.pose.orientation.x = 0.0;
                goalMsg.goal.target_pose.pose.orientation.y = 0.0;
                goalMsg.goal.target_pose.pose.orientation.z = currentOrientationZ_;
                goalMsg.goal.target_pose.pose.orientation.w = currentOrientationW_;
                move_base_goal_pub_.publish(goalMsg);

                goalSent = true;
            }

            if(abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0)
            {
                actionlib_msgs::GoalID cancelMsg;
                move_base_goal_cancel_pub_.publish(cancelMsg);
                finiteStateMachineState_ = 1;
                ROS_WARN("CANCELLED");
            }
        case 7:
            if(goalSent == false)
            {
                move_base_msgs::MoveBaseActionGoal goalMsg;
                goalMsg.goal.target_pose.header.frame_id = "map";
                goalMsg.goal.target_pose.pose.position.x = 0.0;
                goalMsg.goal.target_pose.pose.position.y = 0.0;
                goalMsg.goal.target_pose.pose.position.z = 0.0;
                goalMsg.goal.target_pose.pose.orientation.x = 0.0;
                goalMsg.goal.target_pose.pose.orientation.y = 0.0;
                goalMsg.goal.target_pose.pose.orientation.z = currentOrientationZ_;
                goalMsg.goal.target_pose.pose.orientation.w = currentOrientationW_;
                move_base_goal_pub_.publish(goalMsg);

                goalSent = true;
            }

            if(abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0)
            {
                actionlib_msgs::GoalID cancelMsg;
                move_base_goal_cancel_pub_.publish(cancelMsg);
                finiteStateMachineState_ = 1;
                ROS_WARN("CANCELLED");
            }
        break;

    }

    
  /*  
  float minRangeFront = findMinDist(45.0, 90.0);
   float minRangeBack = findMinDist(135.0, 90.0);

    if(minRangeBack < safeZoneDist && minRangeFront < safeZoneDist)
    {
        cmdVel3Msg_.linear.x = 0.0;
        cmdVel3Msg_.angular.z = 0.5;
        cmd_vel_pub_.publish(cmdVel3Msg_);
    }
    else if(minRangeFront < safeZoneDist)
    {
         cmdVel3Msg_.linear.x = -0.02;
         cmdVel3Msg_.angular.z = 0.0;
         cmd_vel_pub_.publish(cmdVel3Msg_);
         ROS_INFO("Current controller is Automation");
    }
    else if(minRangeBack < safeZoneDist)
    {
       cmdVel3Msg_.linear.x = 0.02;
       cmdVel3Msg_.angular.z = 0.0;
       cmd_vel_pub_.publish(cmdVel3Msg_);
    }
    else if(minRangeBack < bufferZoneDist || minRangeFront < bufferZoneDist && (minRangeBack >= safeZoneDist && minRangeFront >= safeZoneDist))
    {
        currentController = 2;
        
        float speed1 = std::min(minRangeBack,minRangeFront);
        float bufferSpeed = (((speed1-safeZoneDist)/(bufferZoneDist-safeZoneDist))*(maxSpeed-slowSpeed)+slowSpeed);
        updateSuperCommand(currentController,bufferSpeed,1.0,1.0);
        cmd_vel_pub_.publish(cmdVel2Msg_);
        ROS_INFO("Current controller is PS3");
    }
    else
    { 
if(goalSent == false)
{
        move_base_msgs::MoveBaseActionGoal goalMsg;
        goalMsg.goal.target_pose.header.frame_id = "map";
        goalMsg.goal.target_pose.pose.position.x = 0.0;
        goalMsg.goal.target_pose.pose.position.y = 0.0;
        goalMsg.goal.target_pose.pose.position.z = 0.0;
        goalMsg.goal.target_pose.pose.orientation.x = 0.0;
        goalMsg.goal.target_pose.pose.orientation.y = 0.0;
        goalMsg.goal.target_pose.pose.orientation.z = 0.382499;
        goalMsg.goal.target_pose.pose.orientation.w = 0.923955;
        move_base_goal_pub_.publish(goalMsg);

        goalSent = true;
}
        currentController = 2;
        updateSuperCommand(currentController,maxSpeed,2.0,1.0);
        cmd_vel_pub_.publish(cmdVel2Msg_);
        ROS_INFO("Current controller is PS3");
         
        Quaternion q;       
        q = eulerToQuat(2.0,0.0,0.0);
        ROS_WARN("q.x %f", q.x);
        ROS_WARN("q.y %f", q.y);
        ROS_WARN("q.z %f", q.z);
        ROS_WARN("q.w %f", q.w);
    }
*/

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
