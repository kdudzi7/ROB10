
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
    Quaternion q = {0.0, 0.0, currentOrientationZ_, currentOrientationW_};  
    EulerAngles e;
    e = ToEulerAngles(q);
   // ROS_WARN("aaa %f", e.yaw);
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
   	nh_.setParam("/drivingDirection", drivingDirection);
    nh_.setParam("/ps3LinVel", cmdVel2Msg_.linear.x);
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
    float minRange = 3.5;
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
            //ROS_INFO("i %f", scanData[i][1]);
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
			chunkWeights[3] = 0.0;
			chunkWeights[4] = 0.0;
		}
		else
		{
			chunkWeights[0] = 0.0;
			chunkWeights[7] = 0.0;
			chunkWeights[3] = 1.0;
			chunkWeights[4] = 1.0;
		}
		chunkValues[x] = 0.0;
        for(int i = 45 * x; i < 45 * (x+1); i++)
        {
			if(scanData[i][1] < 0.4 && scanData[i][2] == 0)
			{            
				chunkValues[x] = chunkValues[x] + exp(exp(0.4 - scanData[i][1]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValues[x] = chunkValues[x] * chunkWeights[x];
		//ROS_INFO("Sum between %i and %i deg = %f", (45*x), (45*(x+1)), chunkValues[x]);
			collectedChunk_ = collectedChunk_ + chunkValues[x];
    }
	//ROS_INFO("CHUNK SUM IS %f", collectedChunk_);
	



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
			if(scanData[i][1] < 0.4 && scanData[i][2] == 0)
			{            
				chunkValuesTurnedRight[x] = chunkValuesTurnedRight[x] + exp(exp(0.4 - scanDataTurnedRight[i]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValuesTurnedRight[x] = chunkValuesTurnedRight[x] * chunkWeights[x];

			collectedChunkTurnedRight_ = collectedChunkTurnedRight_ + chunkValuesTurnedRight[x];
    }
	//ROS_INFO("CHUNK SUM TURNED RIGHT IS %f", collectedChunkTurnedRight_);
	


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
			if(scanData[i][1] < 0.4 && scanData[i][2] == 0)
			{            
				chunkValuesTurnedLeft[x] = chunkValuesTurnedLeft[x] + exp(exp(0.4 - scanDataTurnedLeft[i]));
			}
           // ROS_INFO("i %f", scanData[i][0]);
        }
		chunkValuesTurnedLeft[x] = chunkValuesTurnedLeft[x] * chunkWeights[x];

			collectedChunkTurnedLeft_ = collectedChunkTurnedLeft_ + chunkValuesTurnedLeft[x];
    }
	//ROS_INFO("CHUNK SUM TURNED LEFT IS %f", collectedChunkTurnedLeft_);

	


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

	void Turtlebot3Supervisor::timerCallback(const ros::TimerEvent& event)
	{
		ROS_WARN("TIMER");
		breakoutPossible_ = true;
	}

	void Turtlebot3Supervisor::timerCallback2(const ros::TimerEvent& event)
	{
            ROS_WARN("BEEP");
	        cout << "\a" << flush;
	}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Supervisor::controlLoop()
{
    minRangeFront_ = findMinDist(35.0, 70.0);
    minRangeBack_ = findMinDist(215.0, 70.0);
	//minRange_ = findMinDist(0.0, 360.0);
	calculateChunks(0.0);
	float angularVelocity;
	float linearVelocity;
    float bufferSpeed;
	float maxSpeed = 0.0;
	auto when_started = clock_type::now(); 
   	auto target_time = when_started + 100ms;
    auto now = clock_type::now();
	nh_.getParam("/testCondition", testCondition);
    if(timerStarted_ == false)
    {
        if(finiteStateMachineState_ == 8 || finiteStateMachineState_ == 9)
        {
            timer2_ = nh_.createTimer(ros::Duration(0.5), &Turtlebot3Supervisor::timerCallback2, false);
            timerStarted_ = true;
        }
        else
        {
            timer2_.stop();
        }
    }
	//ROS_WARN("Break %i", breakoutPossible_);
	//ROS_INFO("Test condition %i", testCondition);
	//ROS_INFO("minRangeF %f", minRangeFront_);
	//ROS_INFO("minRangeB %f", minRangeBack_);
	//ROS_INFO("Driving direction %i", drivingDirection);

    switch(finiteStateMachineState_)
    {

		//Free Driving State
        case 1:
	        //cout << "\a";
            updateSuperCommand(maxSpeed_, 1.0);
            cmd_vel_pub_.publish(cmdVel2Msg_);
            timerStarted_ = false;

            //ROS_INFO("Free Drive zone, FSM is %i", finiteStateMachineState_);

		
            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
            {
				ROS_INFO("Moving to State 2");
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 3");
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 4");
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
				ROS_INFO("Moving to State 5");
                finiteStateMachineState_ = 5;
            }  
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < 0.1) && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
					ROS_INFO("Moving to State 7");
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					ROS_INFO("Moving to State 8");
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					ROS_INFO("Moving to State 9");
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

        break;

		//Buffer zone is violated --- reduce speed 
		//TODO: Different behaviour between wall and obstacle?
        case 2:
            timerStarted_ = false;
            {                
                float speed1 = std::min(minRangeBack_,minRangeFront_);
                bufferSpeed = (((speed1 - safeZoneDist_) / (bufferZoneDist_ - safeZoneDist_)) * (maxSpeed_ - slowSpeed_) + slowSpeed_);
                updateSuperCommand(bufferSpeed, 1.0);
                cmd_vel_pub_.publish(cmdVel2Msg_);
             //   ROS_INFO("Buffer zone, FSM is %i", finiteStateMachineState_);
            }

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
            {
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 3");
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 4");
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
				ROS_INFO("Moving to State 5");
                finiteStateMachineState_ = 5;
            }  
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < 0.1) && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
					ROS_INFO("Moving to State 7");
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					ROS_INFO("Moving to State 8");
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					ROS_INFO("Moving to State 9");
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

        break;

		//Safety Zone in the front is violated --- move backwards
        case 3:
             cmdVel3Msg_.linear.x = std::min(-0.02, cmdVel2Msg_.linear.x);
             
             cmdVel3Msg_.angular.z = cmdVel2Msg_.angular.z;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
            // ROS_INFO("Safe zone front, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
            {
				ROS_INFO("Moving to State 2");
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 4");
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
				ROS_INFO("Moving to State 5");
                finiteStateMachineState_ = 5;
            }  
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < 0.1))// && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
					ROS_INFO("Moving to State 7");
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					ROS_INFO("Moving to State 8");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					ROS_INFO("Moving to State 9");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

			/*if((abs(cmdVel2Msg_.angular.z) >= 1.0) && breakoutPossible_ == true && testCondition == 2)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }
			else if((abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0) && breakoutPossible_ == true && testCondition == 3)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }*/
               
        break;

		//Safety Zone in the back is violated --- move forward
        case 4:
             cmdVel3Msg_.linear.x = std::max(0.02, cmdVel2Msg_.linear.x);
             cmdVel3Msg_.angular.z = cmdVel2Msg_.angular.z;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
            // ROS_INFO("Safe zone back, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
            {
				ROS_INFO("Moving to State 2");
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 3");
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
				ROS_INFO("Moving to State 5");
                finiteStateMachineState_ = 5;
            }  
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < 0.1))// && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
					ROS_INFO("Moving to State 7");
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					ROS_INFO("Moving to State 8");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					ROS_INFO("Moving to State 9");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

			/*if((abs(cmdVel2Msg_.angular.z) >= 1.0) && breakoutPossible_ == true && testCondition == 2)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }
			else if((abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0) && breakoutPossible_ == true && testCondition == 3)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }*/
                   
        break;
       
		//Safety Zone if both front and back sensors hit --- turning until front or back are free to drive
		case 5:
            cmdVel3Msg_.linear.x = 0.0;
            cmdVel3Msg_.angular.z = 0.5;
            cmd_vel_pub_.publish(cmdVel3Msg_);            
           // ROS_INFO("Safe zone front and back, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
            {
				ROS_INFO("Moving to State 2");
                finiteStateMachineState_ = 2;                
            }
            else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 3");
                finiteStateMachineState_ = 3;                
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
            {
				ROS_INFO("Moving to State 4");
                finiteStateMachineState_ = 4;
            }
            else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
            {
                finiteStateMachineState_ = 5;
            }  
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < 0.1))// && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
					ROS_INFO("Moving to State 7");
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					ROS_INFO("Moving to State 8");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					ROS_INFO("Moving to State 9");
					//breakoutPossible_ = false;
					//timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    //timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

			/*if((abs(cmdVel2Msg_.angular.z) >= 1.0) && breakoutPossible_ == true && testCondition == 2)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }
			else if((abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0) && breakoutPossible_ == true && testCondition == 3)
            {
				breakoutPossible_ = false;
				timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                ROS_WARN("BROKEN OUT! Remaining in state");
            }*/
           
        break;
        
	

		//Teleoperation Obstacle Avoidance
		case 7:

			
		if(chunkWeights[1] > 0.0)
		{
			chunkWeights[1] = -0.1;
			chunkWeights[2] = 0.1;
			chunkWeights[5] = -0.1;
			chunkWeights[6] = 0.1;
		}
		else
		{
			chunkWeights[1] = 0.1;
			chunkWeights[2] = -0.1;
			chunkWeights[5] = 0.1;
			chunkWeights[6] = -0.1;
		}
           
 		updateSuperCommand(0.1, 1.0);

		/*when_started = clock_type::now(); 
    	target_time = when_started + 100ms;
	    for (int i = 0; i < 10; i++) {
			cout << "\a";
   			 cout << "Beep" << endl;
		    std::this_thread::sleep_until(target_time);
		    target_time += 100ms;
    	}
	    now = clock_type::now();
	    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - when_started).count() << "ms\n";
*/

		cmd_vel_pub_.publish(cmdVel2Msg_); 

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
        {
			ROS_INFO("Moving to State 2");
            finiteStateMachineState_ = 2;                
        }
        else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
        {
			ROS_INFO("Moving to State 3");
            finiteStateMachineState_ = 3;                
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
        {
			ROS_INFO("Moving to State 4");
            finiteStateMachineState_ = 4;
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
        {
			ROS_INFO("Moving to State 5");
            finiteStateMachineState_ = 5;
        }  
		else if(collectedChunk_ > 0.1 || collectedChunk_ < 0.1)
		{
            finiteStateMachineState_ = 7;                			
		}

		break;

		//Semi Autonomy Obstacle Avoidance
		case 8:
		angularVelocity = std::min((abs(collectedChunk_ / minRangeFront_) * 2.84), 0.34);
			
		if(chunkWeights[1] > 0.0)
		{
			chunkWeights[1] = -0.1;
			chunkWeights[2] = 0.1;
			chunkWeights[5] = -0.1;
			chunkWeights[6] = 0.1;
		}
		else
		{
			chunkWeights[1] = 0.1;
			chunkWeights[2] = -0.1;
			chunkWeights[5] = 0.1;
			chunkWeights[6] = -0.1;
		}
           
 		updateSuperCommand(0.1, 1.0);

		if(abs(collectedChunkTurnedLeft_) < abs(collectedChunk_) && abs(collectedChunkTurnedLeft_) < abs(collectedChunkTurnedRight_))
		{
			cmdVel3Msg_.linear.x = cmdVel2Msg_.linear.x;
			cmdVel3Msg_.angular.z = angularVelocity;
				//ROS_WARN("Turning left");
		}
		else if(abs(collectedChunkTurnedRight_) < abs(collectedChunk_) && abs(collectedChunkTurnedRight_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = cmdVel2Msg_.linear.x;
		    cmdVel3Msg_.angular.z = -angularVelocity;
				//ROS_WARN("Turning right");
		}
		else if(abs(collectedChunk_) < abs(collectedChunkTurnedRight_) && abs(collectedChunk_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = cmdVel2Msg_.linear.x;
		    cmdVel3Msg_.angular.z = 0.0;
				//ROS_WARN("Keeping Straight");
		}

		cmd_vel_pub_.publish(cmdVel3Msg_); 

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                timer_.stop();
                breakoutPossible_ = true;
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
        {
			ROS_INFO("Moving to State 2");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 2;                
        }
        else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
        {
			ROS_INFO("Moving to State 3");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 3;                
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
        {
			ROS_INFO("Moving to State 4");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 4;
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
        {
			ROS_INFO("Moving to State 5");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 5;
        }  
		else if(collectedChunk_ > 0.1 || collectedChunk_ < 0.1)
		{
            finiteStateMachineState_ = 8;                			
		}

		if((abs(cmdVel2Msg_.angular.z) >= 1.0) && breakoutPossible_ == true)
        {
            finiteStateMachineState_ = 2;
			breakoutPossible_ = false;
			timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
            ROS_WARN("BROKEN OUT! Moving to State2");
        }
		break;


		//Full Autonomy Obstacle Avoidance
		case 9:

          

		angularVelocity = std::min((abs(collectedChunk_ / minRangeFront_) * 2.84), 0.34);
			
		if(chunkWeights[1] > 0.0)
		{
			chunkWeights[1] = -0.1;
			chunkWeights[2] = 0.1;
			chunkWeights[5] = -0.1;
			chunkWeights[6] = 0.1;
		}
		else
		{
			chunkWeights[1] = 0.1;
			chunkWeights[2] = -0.1;
			chunkWeights[5] = 0.1;
			chunkWeights[6] = -0.1;
		}
           
 		//updateSuperCommand(0.04, 1.0);

		if(abs(collectedChunkTurnedLeft_) < abs(collectedChunk_) && abs(collectedChunkTurnedLeft_) < abs(collectedChunkTurnedRight_))
		{
			cmdVel3Msg_.linear.x = 0.1;//cmdVel2Msg_.linear.x;
			cmdVel3Msg_.angular.z = angularVelocity;
				//ROS_WARN("Turning left");
		}
		else if(abs(collectedChunkTurnedRight_) < abs(collectedChunk_) && abs(collectedChunkTurnedRight_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = 0.1;//cmdVel2Msg_.linear.x;
		    cmdVel3Msg_.angular.z = -angularVelocity;
				//ROS_WARN("Turning right");
		}
		else if(abs(collectedChunk_) < abs(collectedChunkTurnedRight_) && abs(collectedChunk_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = 0.1;//cmdVel2Msg_.linear.x;
		    cmdVel3Msg_.angular.z = 0.0;
				//ROS_WARN("Keeping Straight");
		}

		cmd_vel_pub_.publish(cmdVel3Msg_); 

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
				ROS_INFO("Moving to State 1");
                timer_.stop();
                breakoutPossible_ = true;
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
        {
			ROS_INFO("Moving to State 2");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 2;                
        }
        else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
        {
			ROS_INFO("Moving to State 3");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 3;                
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
        {
            ROS_INFO("Range back %f", minRangeBack_);
            ROS_INFO("Range front %f", minRangeFront_);
			ROS_INFO("Moving to State 4");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 4;
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
        {
			ROS_INFO("Moving to State 5");
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 5;
        }  
		else if(collectedChunk_ > 0.1 || collectedChunk_ < 0.1)
		{
            finiteStateMachineState_ = 9;                			
		}

		if((abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0) && breakoutPossible_ == true)
        {
            finiteStateMachineState_ = 2;
			breakoutPossible_ = false;
			timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
            ROS_WARN("BROKEN OUT! Moving to State2");
        }
		break;
	}
	

    	//Path Planning to send a goal
		/*
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
        break;*/
 

      return true;
}

	bool Turtlebot3Supervisor::breakoutPossible_ = true;
    bool Turtlebot3Supervisor::timerStarted_ = false;

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
