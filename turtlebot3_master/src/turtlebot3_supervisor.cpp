
/*******************************************************************************
* 
*******************************************************************************/

/* Authors: Adrian Herskind & Karolina Dudzinska */

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
    
    // initialize test parameter
    numberOfBreakouts_ = 0;

    // initialize state machine
    finiteStateMachineState_ = 1;

    // initialize publishers
    super_pub_ = nh_.advertise<turtlebot3_master::Super>("super_cmd", 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // initialize subscribers
    cmd_vel2_sub_ = nh_.subscribe("cmd_vel2", 10, &Turtlebot3Supervisor::cmdVel2MsgCallBack, this);
    laser_scan_eval_sub_ = nh_.subscribe("scan_eval", 10, &Turtlebot3Supervisor::laserScanEvalMsgCallBack, this);

    ROS_WARN("Welcome.");
    ROS_WARN("Number of Break outs is reset");
    ROS_WARN("Have fun!");

    // Set A default starting value for drivingDirection
    nh_.setParam("/drivingDirection", 1);
    return true;
}

/*
*When input from the PS3 Controller comes, the system saves the linear and angular velocity in variables.
*If the linear input velocity is below 0 then the driving direction is set to backwards, else to forward
*Driving Direction is updated.
*/
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

/**
*When the ScanData message is received, the data is saved in an array
*/
void Turtlebot3Supervisor::laserScanEvalMsgCallBack(const turtlebot3_master::ScanData::ConstPtr &msg)
{
    for(int i = 0; i < 360; i++)
    {
        scanData[i][0] = msg->angles[i];
        scanData[i][1] = msg->ranges[i];
        scanData[i][2] = msg->type[i];
    }
}

/**
* Publishes a message with the new linear and angular velocity limits
*/
void Turtlebot3Supervisor::updateSuperCommand(float linearVelLimit, float angularVelLimit)
{
    turtlebot3_master::Super super_cmd;

    super_cmd.linearVelLimit = linearVelLimit;
    super_cmd.angularVelLimit = angularVelLimit;

    super_pub_.publish(super_cmd);
}

/*
* Returns the shortest distance from a set of scan data. The parameters describe the left angle and how much many angles should be spanned clockwise.
*/
float Turtlebot3Supervisor::findMinDist(float startLeftAngle, float angle)
{
    //Maximum value
    float minRange = 3.5;

    //For invalid input, the function returns 666.0
    //IMPROVE: Ideally it should return an error message
    if(startLeftAngle >= 360.0 || angle > 360.0 || startLeftAngle < 0.0 || angle <= 0.0)
    {
        return 666.0;
    }

    //Case 1: If the angles we consider do not cross the 0 - it is a mathematical thing
    if((startLeftAngle - angle) > 0)
    {
        //Iterate over all angles --- if range value is smaller than current minimum, set this range as minimum
        for(int i = (startLeftAngle - angle); i < startLeftAngle; i++)
        {
            if(scanData[i][0] < startLeftAngle && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }
        }
    }
    //Case 2: If angles we consider do cross the 0
    else
    {
        //First section of the angles we consider - iterate over all angles from 0 counter clockwise - if range value is smaller than current minimum, set this range as minimum
        for(int i = 0; i < startLeftAngle; i++)
        {
            if(scanData[i][0] < startLeftAngle && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }
        }

        for(int i = (360 + startLeftAngle - angle); i < 360; i++)
        {
            if(scanData[i][0] > (360 + startLeftAngle - angle) && scanData[i][1] < minRange)
            {
                minRange = scanData[i][1];
            }  
        }
    }

    return minRange;
}

/*
*Calculates chunk values for 8 chunks, 360 degree scan is split up in 45 degree chunks, starts at 0 degrees
*For each chunk we sum a value calculated from the range --> This gives a chunkValue for each chunk, weighted
*We calculate a collectedChuk, which is the sum of all 8 chunkValues
*Then we simulate a 10 degree left and right turn and calculate chunk sums for these as well.
*We only consider ranges closer than 0.4
*/
void Turtlebot3Supervisor::calculateChunks()
{
	collectedChunk_ = 0.0;
    for(int x = 0; x < 8; x++)
    {
        //Adapting chunkWeights to driving direction
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
        }
		chunkValues[x] = chunkValues[x] * chunkWeights[x];
		//ROS_INFO("Sum between %i and %i deg = %f", (45*x), (45*(x+1)), chunkValues[x]);
			collectedChunk_ = collectedChunk_ + chunkValues[x];
    }
	//ROS_INFO("CHUNK SUM IS %f", collectedChunk_);
	
    //Calculation for everything turned 10 degrees to right
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
	
    //Calculation for everything turned 10 degrees to left
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


/*
* When this timer runs out, breakout is again possible
*/
void Turtlebot3Supervisor::timerCallback(const ros::TimerEvent& event)
{
	breakoutPossible_ = true;
}

/*
* When this timer runs out, a system sound is played
*/
void Turtlebot3Supervisor::timerCallback2(const ros::TimerEvent& event)
{
    std::cout << "\a" << std::flush;
}

/*******************************************************************************
* Control Loop function - here everything fun happens
*******************************************************************************/
bool Turtlebot3Supervisor::controlLoop()
{
    //What is the closest object in front and back
    minRangeFront_ = findMinDist(35.0, 70.0);
    minRangeBack_ = findMinDist(215.0, 70.0);

    //We calculate the chunks
	calculateChunks();

	float angularVelocity;
	float linearVelocity;
    float bufferSpeed;

    //What test condition are we in? 1=Teleoperation, 2=SharedControl, 3=FullAutonomy
	nh_.getParam("/testCondition", testCondition);

    //If the timer is not started, we start it in case we are in one of the (semi-)autonomous states (8,9), else it gets stopped
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

    //LeftOver messages from Debug
	//ROS_WARN("Break %i", breakoutPossible_);
	//ROS_INFO("Test condition %i", testCondition);
	//ROS_INFO("minRangeF %f", minRangeFront_);
	//ROS_INFO("minRangeB %f", minRangeBack_);
	//ROS_INFO("Driving direction %i", drivingDirection);

    //FSM
    switch(finiteStateMachineState_)
    {
		//Free Driving State --- Operator drives freeely with speed limits
        case 1:
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
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < -0.1) && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

        break;

		//Buffer zone is violated --- reduce speed 
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
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
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
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < -0.1) && breakoutPossible_ == true)
			{
				if(testCondition == 1)
				{
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
					breakoutPossible_ = false;
					timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
                    timer_.start();
		            finiteStateMachineState_ = 9;       
				}	
			}

        break;

		//Safety Zone in the front is violated --- move backwards with low speed or operator input, whichever is highest
        case 3:
             cmdVel3Msg_.linear.x = std::min(-0.02, cmdVel2Msg_.linear.x);
             
             cmdVel3Msg_.angular.z = cmdVel2Msg_.angular.z;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
            // ROS_INFO("Safe zone front, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
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
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < -0.1))
			{
				if(testCondition == 1)
				{
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
		            finiteStateMachineState_ = 9;       
				}	
			}

			
               
        break;

		//Safety Zone in the back is violated --- move forward with low speed or operator input, whichever is highest
        case 4:
             cmdVel3Msg_.linear.x = std::max(0.02, cmdVel2Msg_.linear.x);
             cmdVel3Msg_.angular.z = cmdVel2Msg_.angular.z;
             cmd_vel_pub_.publish(cmdVel3Msg_);            
            // ROS_INFO("Safe zone back, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
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
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < -0.1))
			{
				if(testCondition == 1)
				{
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
		            finiteStateMachineState_ = 9;       
				}	
			}


                   
        break;
       
		//Safety Zone if both front and back sensors hit --- turning until front or back are free to drive
		case 5:
            cmdVel3Msg_.linear.x = 0.0;
            cmdVel3Msg_.angular.z = 0.5;
            cmd_vel_pub_.publish(cmdVel3Msg_);            
           // ROS_INFO("Safe zone front and back, FSM is %i", finiteStateMachineState_);

            if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
            {
                finiteStateMachineState_ = 1;
            }
            else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
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
		    else if((collectedChunk_ > 0.1 || collectedChunk_ < -0.1))
			{
				if(testCondition == 1)
				{
		            finiteStateMachineState_ = 7;       					
				}
				else if(testCondition == 2)
				{
		            finiteStateMachineState_ = 8;       
				}
				else if(testCondition == 3)
				{
		            finiteStateMachineState_ = 9;       
				}	
			} 
        break;
        
	
		//Teleoperation Obstacle Avoidance - ChunkWeights get oscillated - User drives freely on low speed
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

		cmd_vel_pub_.publish(cmdVel2Msg_); 

        if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
        {
            finiteStateMachineState_ = 1;
        }
        else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
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
		else if(collectedChunk_ > 0.1 || collectedChunk_ < -0.1)
		{
            finiteStateMachineState_ = 7;                			
		}

		break;

		//Semi Autonomy Obstacle Avoidance - Chunk Weights get oscillated - System checks which direction brings chunkWeights closer to 0.0, turns that way - User controls Linear Velocity
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
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 1;
        }
        else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 2;                
        }
        else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 3;                
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 4;
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 5;
        }  
		else if(collectedChunk_ > 0.1 || collectedChunk_ < -0.1)
		{
            finiteStateMachineState_ = 8;                			
		}

        //If the user applies angular velocity, control is transfered back. Timer is started to prevent immediate catchback
		if((abs(cmdVel2Msg_.angular.z) >= 1.0) && breakoutPossible_ == true)
        {
            finiteStateMachineState_ = 2;
			breakoutPossible_ = false;
			timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
            numberOfBreakouts_++;
            ROS_WARN("BROKEN OUT! Counter: %i", numberOfBreakouts_);
        }
		break;


		//Full Autonomy Obstacle Avoidance - Chunk Weights get oscillated - System checks which direction brings chunkWeights closer to 0.0, turns that way - System also controls Linear Velocity
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
           
		if(abs(collectedChunkTurnedLeft_) < abs(collectedChunk_) && abs(collectedChunkTurnedLeft_) < abs(collectedChunkTurnedRight_))
		{
			cmdVel3Msg_.linear.x = 0.1;
			cmdVel3Msg_.angular.z = angularVelocity;
				//ROS_WARN("Turning left");
		}
		else if(abs(collectedChunkTurnedRight_) < abs(collectedChunk_) && abs(collectedChunkTurnedRight_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = 0.1;
		    cmdVel3Msg_.angular.z = -angularVelocity;
				//ROS_WARN("Turning right");
		}
		else if(abs(collectedChunk_) < abs(collectedChunkTurnedRight_) && abs(collectedChunk_) < abs(collectedChunkTurnedLeft_))
		{
			cmdVel3Msg_.linear.x = 0.1;
		    cmdVel3Msg_.angular.z = 0.0;
				//ROS_WARN("Keeping Straight");
		}

		cmd_vel_pub_.publish(cmdVel3Msg_); 

        if(collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1 && minRangeFront_ > bufferZoneDist_ && minRangeBack_ > bufferZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 1;
        }
        else if((minRangeBack_ < bufferZoneDist_ || minRangeFront_ < bufferZoneDist_) && minRangeBack_ >= safeZoneDist_ && minRangeFront_ >= safeZoneDist_ && collectedChunk_ <= 0.1 && collectedChunk_ >= -0.1)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 2;                
        }
        else if(minRangeFront_ < safeZoneDist_ && minRangeBack_ > safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 3;                
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ > safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 4;
        }
        else if(minRangeBack_ < safeZoneDist_ && minRangeFront_ < safeZoneDist_)
        {
            timer_.stop();
            breakoutPossible_ = true;
            finiteStateMachineState_ = 5;
        }  
		else if(collectedChunk_ > 0.1 || collectedChunk_ < -0.1)
		{
            finiteStateMachineState_ = 9;                			
		}

        //If the user applies angular or linear velocity, control is transfered back. Timer is started to prevent immediate catchback
		if((abs(cmdVel2Msg_.linear.x) > 0.0 || abs(cmdVel2Msg_.angular.z) > 0.0) && breakoutPossible_ == true)
        {
            finiteStateMachineState_ = 2;
			breakoutPossible_ = false;
			timer_ = nh_.createTimer(ros::Duration(5.0), &Turtlebot3Supervisor::timerCallback, true);
            numberOfBreakouts_++;
            ROS_WARN("BROKEN OUT! Counter: %i", numberOfBreakouts_);
        }
		break;
	}
	
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
