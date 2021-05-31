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

/* Authors: Adrian Herskind & Karolina Dudzinska */

#ifndef TURTLEBOT3_SUPERVISOR_H_
#define TURTLEBOT3_SUPERVISOR_H_
#define _USE_MATH_DEFINES

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_master/Super.h>
#include <turtlebot3_master/ScanData.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2


class Turtlebot3Supervisor
{
    public:
        Turtlebot3Supervisor();
        ~Turtlebot3Supervisor();
        bool init();
        bool controlLoop();
	    static bool breakoutPossible_;
        static bool timerStarted_;
	    static void timerCallback(const ros::TimerEvent& event);
	    static void timerCallback2(const ros::TimerEvent& event);
    private:
        // ROS NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        // ROS Topic Publishers
        ros::Publisher super_pub_;
        ros::Publisher cmd_vel_pub_;
            
        // ROS Topic Subscribers
        ros::Subscriber cmd_vel2_sub_;
        ros::Subscriber joy_sub_;
        ros::Subscriber laser_scan_eval_sub_;

        geometry_msgs::Twist cmdVel2Msg_;
        geometry_msgs::Twist cmdVel3Msg_;

        int finiteStateMachineState_ = 0;

        float maxSpeed_ = 0.22;
        float slowSpeed_ = 0.08;
        float safeZoneDist_ = 0.2;
        float bufferZoneDist_ = 0.8;
        float minRangeFront_, minRangeBack_;
    	int drivingDirection = 0;
	
	    float chunkValues[8] = {};
    	float chunkWeights[8] = {1.0, -0.1, -0.0, 0.2, 0.2, -0.1, -0.0, 1.0};
    	float collectedChunk_ = 0.0;

	    float chunkValuesTurnedLeft[8] = {};
    	float collectedChunkTurnedLeft_ = 0.0;

	    float chunkValuesTurnedRight[8] = {};
    	float collectedChunkTurnedRight_ = 0.0;

	    // 1=Feedback Assisted Teleoperation 2=Shared Control 3=Full Autonomy
	    int testCondition = 2;

        int numberOfBreakouts_ = 0;

        float scanData[360][3] = {0};

	    ros::Timer timer_;
    	ros::Timer timer2_;

        // Function prototypes
        float findMinDist(float startLeftAngle, float angle);
        void calculateChunks(float startAngle);

        void updateSuperCommand(float linearVelLimit, float angularVelLimit);

        void cmdVel2MsgCallBack(const geometry_msgs::Twist::ConstPtr &msg);
        void laserScanEvalMsgCallBack(const turtlebot3_master::ScanData::ConstPtr &msg);
};
#endif // TURTLEBOT3_SUPERVISOR_H_
