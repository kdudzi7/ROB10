
#include <math.h>

#include <boost/thread.hpp>

#include "ros/ros.h"

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
//#include <metralabs_msgs/IDAndFloat.h>
//#include <metralabs_msgs/ScitosG5Bumper.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>




// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19



#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)


/*class TeleopPS3
{
public:
	TeleopPS3();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_schunk_;

	ros::Subscriber joy_sub_;

	ros::Publisher base_vel_pub_;
	ros::Publisher arm_vel_pub_;
	ros::Publisher ack_all_joints_pub_;
	ros::Publisher arm_emergency_pub_;
	ros::Publisher joints_position_pub_;
	ros::Publisher gripper_pub_;
	ros::Publisher smach_enable_pub_;
	ros::Publisher bumper_reset_pub_;
	ros::Publisher move_base_cancel_pub_;

	const static int axis_speed = PS3_AXIS_STICK_LEFT_UPWARDS;
	const static int axis_turn = PS3_AXIS_STICK_LEFT_LEFTWARDS;
	const static int axis_turbo = PS3_AXIS_BUTTON_REAR_RIGHT_2;
	const static int axis_turbo_second = PS3_AXIS_BUTTON_REAR_LEFT_2;
	const static int button_deadman = PS3_BUTTON_REAR_RIGHT_1;
	const static int button_deadman_second = PS3_BUTTON_REAR_LEFT_1;
	const static int button_turbo = PS3_BUTTON_REAR_RIGHT_2;

	const static int button_modifier_config = PS3_BUTTON_START;
	const static int button_joints_ack_all = PS3_BUTTON_ACTION_SQUARE;
	const static int button_joints_go_pose = PS3_BUTTON_ACTION_TRIANGLE;

	const static int button_emergency = PS3_BUTTON_ACTION_CIRCLE;

	const static int button_modifier_nonarm_config = PS3_BUTTON_SELECT;
	const static int button_smach_enable = PS3_BUTTON_ACTION_SQUARE;
	const static int button_smach_disable = PS3_BUTTON_ACTION_CROSS;
	const static int button_bumper_reset = PS3_BUTTON_ACTION_TRIANGLE;
	const static int button_move_base_cancel = PS3_BUTTON_ACTION_CIRCLE;

	const static int axis_arm_pitch = PS3_AXIS_STICK_RIGHT_UPWARDS;
	const static int axis_arm_yaw = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
	const static int axis_gripper_close = PS3_AXIS_BUTTON_CROSS_DOWN;
	const static int axis_gripper_open = PS3_AXIS_BUTTON_CROSS_UP;

	constexpr static double speed = 0.3; // .2 .5
	constexpr static double turn = 0.7; // .5 1
	constexpr static double joint_speed_pitch = 0.87; // joint 4
	constexpr static double joint_speed_yaw = 0.43; // joint 0
//	constexpr static double gripper_speed = 0.08;
	constexpr static double gripper_step = 0.01;
	constexpr static double gripper_step_duration = 0.05;
	constexpr static double velocity_gate_min = 0.05;
	constexpr static double bumper_reset_mute_duration = 0.5;
};


TeleopPS3::TeleopPS3() :
		nh_schunk_("schunk")
{
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopPS3::joyCallback, this);

	base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 3);
	smach_enable_pub_ = nh_.advertise<std_msgs::Bool>("enable_smach", 1);
	bumper_reset_pub_ = nh_.advertise<std_msgs::Empty>("bumper_reset", 1);
	move_base_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

	arm_vel_pub_ = nh_schunk_.advertise<geometry_msgs::Twist>("moveArmVelocity", 3);
	ack_all_joints_pub_ = nh_schunk_.advertise<std_msgs::Empty>("ack_all", 1);
	arm_emergency_pub_ = nh_schunk_.advertise<std_msgs::Empty>("emergency", 1);
	joints_position_pub_ = nh_schunk_.advertise<sensor_msgs::JointState>("move_all_position", 1);
	//gripper_pub_ = nh_schunk_.advertise<metralabs_msgs::IDAndFloat>("move_position", 1);
}

void TeleopPS3::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	/// DEBUGGING

//	int i;
//	for(i=0; i < 16; i++)
//		ROS_INFO_STREAM("axis   " << i << " " << joy->axes[i]);
//	for(i=0; i < joy->buttons.size(); i++)
//		ROS_INFO_STREAM("button " << i << " " << joy->buttons[i]);


	/// base move control

	geometry_msgs::TwistPtr velocity_msg (new geometry_msgs::Twist);
	static bool sent_zero_last_time = false;
	if(joy->buttons[button_deadman] || joy->buttons[button_deadman_second] || joy->buttons[button_turbo])
	{
		double speedfactor = 1 + (-joy->axes[axis_turbo]) + (-joy->axes[axis_turbo_second]); // up to thrice the speed if turbo
		velocity_msg->linear.x = joy->axes[axis_speed] * speed * speedfactor;
		velocity_msg->angular.z = joy->axes[axis_turn] * turn * speedfactor;

		ROS_DEBUG("effective linear x = %f, effective angular z = %f",
				velocity_msg->linear.x, velocity_msg->angular.z);
		// do not repeat zero'd messages on idle to not disturb other publishers
		if(fabs(velocity_msg->linear.x) <= velocity_gate_min &&
				fabs(velocity_msg->angular.z) <= velocity_gate_min)
		{
			velocity_msg->linear.x = 0;
			velocity_msg->angular.z = 0;
			if(sent_zero_last_time == false) {
				sent_zero_last_time = true;
				base_vel_pub_.publish(velocity_msg);
				ROS_DEBUG("Sent zero'd.");
			}
			else
				ROS_DEBUG("Sent nothing.");
		}
		else {
			sent_zero_last_time = false;
			base_vel_pub_.publish(velocity_msg);
			ROS_DEBUG("Sent values.");
		}
	}
	else {
		// leave message zero'd
		// do not repeat zero'd messages on idle to not disturb other publishers
		if(sent_zero_last_time == false) {
			sent_zero_last_time = true;
			base_vel_pub_.publish(velocity_msg);
			ROS_DEBUG("Sent zero'd.");
		}
		else
			ROS_DEBUG("Sent nothing.");
	}


	/// arm move control

	if(joy->buttons[button_emergency] && !joy->buttons[button_modifier_nonarm_config]) {
		// emergency not when pushed with nonarm config
		arm_emergency_pub_.publish(std_msgs::Empty());
	}
	else if(joy->buttons[button_deadman] || joy->buttons[button_deadman_second]) {
		/// arm
		geometry_msgs::TwistPtr arm_msg (new geometry_msgs::Twist);
		arm_msg->angular.y = joy->axes[axis_arm_pitch] * joint_speed_pitch;
		arm_msg->angular.z = -joy->axes[axis_arm_yaw] * joint_speed_yaw;
		arm_vel_pub_.publish(arm_msg);

		/// gripper
		// timed gate
		static ros::Time lastAction = ros::Time(0);
		if(ros::Time::now() - lastAction > ros::Duration(gripper_step_duration) ){
			lastAction = ros::Time::now();

			// static vars
			static float new_gripper_value = 0.068;
			//static metralabs_msgs::IDAndFloatPtr gripper_msg (new metralabs_msgs::IDAndFloat);
			//gripper_msg->id = 5;

			// input
			float close_axis = - joy->axes[axis_gripper_close]; // converted to 0 to 1
			float open_axis  = - joy->axes[axis_gripper_open ]; // converted to 0 to 1

			// logic

			if(!close_axis && open_axis)
				new_gripper_value += gripper_step * open_axis;
			else if(close_axis && !open_axis)
				new_gripper_value -= gripper_step * close_axis;

			new_gripper_value = fmax(new_gripper_value, 0);
			new_gripper_value = fmin(new_gripper_value, 0.068);

			//if(gripper_msg->value != new_gripper_value) {
			//	ROS_INFO_STREAM("new_gripper_value: " << new_gripper_value);
			//	gripper_msg->value = new_gripper_value;
				//gripper_pub_.publish(gripper_msg);
			//}
		}

		/*/
/* gripper
		metralabs_msgs::IDAndFloatPtr gripper_msg (new metralabs_msgs::IDAndFloat);
		gripper_msg->id = 5;
		float close_axis = - joy->axes[axis_gripper_close]; // converted to 0 to 1
		float open_axis  = - joy->axes[axis_gripper_open ]; // converted to 0 to 1
		if(!close_axis && open_axis)
			gripper_msg->value = open_axis * gripper_speed;
		else if(close_axis && !open_axis)
			gripper_msg->value = close_axis * -gripper_speed;
		else
			gripper_msg->value = 0;
		gripper_pub_.publish(gripper_msg);
		*/
	//}

	/// second layer arm buttons

	/*if(joy->buttons[button_modifier_config]) {
		if(joy->buttons[button_joints_ack_all]) {
			ack_all_joints_pub_.publish(std_msgs::Empty());
		}

		static bool button_blocked = false;
		if(joy->buttons[button_joints_go_pose]) {
			if(!button_blocked) {
				button_blocked = true;

				sensor_msgs::JointState::Ptr joint_msg (new sensor_msgs::JointState);
	//			std::string names[] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
	//			double values[] = {0, 0.5, 0.5, 1.5, 0};
	//			joint_msg->name.resize(5);
	//			joint_msg->position.resize(5);
	//			for(int i=0; i<5; i++) {
	//				joint_msg->name[i] = names[i];
	//				joint_msg->position[i] = values[i];
	//			}


//		ARM_FOLDED_POSE = [0, 0.52, 0.52, -1.57, 0]
//		ARM_FOLDED_POSE_NAMES = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
				joint_msg->name = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
				joint_msg->position = {0, 0.5, 0.5, -1.6, 0};
				joint_msg->velocity.assign(5, 0.3);
				joints_position_pub_.publish(joint_msg);
			}
		}
		else
			button_blocked = false;
	}


	/// second layer non arm config buttons

	if(joy->buttons[button_modifier_nonarm_config]) {

		/// smach
		std_msgs::Bool smach_msg;
		if(joy->buttons[button_smach_disable]) {
			smach_msg.data = false;
			smach_enable_pub_.publish(smach_msg);
		}
		else if(joy->buttons[button_smach_enable]) {
			smach_msg.data = true;
			smach_enable_pub_.publish(smach_msg);
		}

		/// bumper
		if(joy->buttons[button_bumper_reset]) {
			// timed gate
			static ros::Time lastAction = ros::Time(0);
			if(ros::Time::now() - lastAction > ros::Duration(bumper_reset_mute_duration) ){
				lastAction = ros::Time::now();
				bumper_reset_pub_.publish(std_msgs::Empty());
			}
		}

		/// move_base
		if(joy->buttons[button_move_base_cancel]) {
			// timed gate
			static ros::Time lastAction = ros::Time(0);
			if(ros::Time::now() - lastAction > ros::Duration(bumper_reset_mute_duration) ){
				lastAction = ros::Time::now();
				move_base_cancel_pub_.publish(actionlib_msgs::GoalID());
			}
		}
	}


}

*/

class TeleopPS3Feedback
{
public:
	typedef sensor_msgs::JoyFeedback::_id_type ID;
	typedef sensor_msgs::JoyFeedback::_intensity_type INTENSITY;

	struct LED
	{
		ID id;
		INTENSITY intensity;
	};


	TeleopPS3Feedback()
	{
		feedback_pub_ = nh_.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 5);

		//bumper_sub_ = nh_.subscribe("/bumper", 3, &TeleopPS3Feedback::bumperCallback, this);
		//smach_enabled_sub_ = nh_.subscribe("/enable_smach", 3, &TeleopPS3Feedback::smachEnabledCallback, this);

		sendLEDs(false, true, false, true);
		sendRumble(0, 0);


		// init led arrays
		for(int id=0; id<4; id++)
		{
			leds_status[id].id = id;
			leds_status[id].intensity = 0;
			leds_knight_rider[id].id = id;
			leds_knight_rider[id].intensity = 0;
		}

		boost::thread(&TeleopPS3Feedback::knightRiderThread, this);

	}

protected:
	/*void bumperCallback(const metralabs_msgs::ScitosG5BumperConstPtr& bumper)
	{
		static bool last_state = false;
		bool current_state = bumper->bumper_pressed;

		if(!last_state && current_state)
		{
			sendRumble(1, 1);
			ros::WallDuration duration;
			duration.fromSec(0.2);
			duration.sleep();
			sendRumble(0, 0);
		}

		leds_status[LED_INDEX_MOTOR_STOP].intensity = bumper->motor_stop ? 1 : 0;
		updateLEDs();

		last_state = current_state;
	}*/

	void smachEnabledCallback(const std_msgs::BoolConstPtr& smach_enabled_msg)
	{
		bool smach_enabled = smach_enabled_msg->data;
		leds_status[LED_INDEX_SMACH_DISABLED].intensity = !smach_enabled ? 1 : 0;
		updateLEDs();
	}


	void rumbleDemo()
	{
		ros::WallDuration duration;
		duration.fromSec(0.4);

		sendRumble(0.5, 0);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(0, 0.5);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(1, 0);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(0, 1);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();


		sendRumble(0.5, 0.5);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(0.5, 1);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(1, 0.5);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();

		sendRumble(1, 1);
		duration.sleep();
		sendRumble(0, 0);
		duration.sleep();
	}


	void sendRumble(INTENSITY rumble_low_freq, INTENSITY rumble_high_freq)
	{
		sensor_msgs::JoyFeedbackArray feedback_array;
		feedback_array.array.push_back(*getFeedbackRumble(0, rumble_low_freq));
		feedback_array.array.push_back(*getFeedbackRumble(1, rumble_high_freq));
		feedback_pub_.publish(feedback_array);
	}


	void sendLEDs(bool led0, bool led1, bool led2, bool led3)
	{
		sensor_msgs::JoyFeedbackArray feedback_array;
		feedback_array.array.push_back(*getFeedbackLED(0, led0));
		feedback_array.array.push_back(*getFeedbackLED(1, led1));
		feedback_array.array.push_back(*getFeedbackLED(2, led2));
		feedback_array.array.push_back(*getFeedbackLED(3, led3));
		feedback_pub_.publish(feedback_array);
	}

	void sendLEDs(LED leds[4])
	{
		sensor_msgs::JoyFeedbackArray feedback_array;
		for(int i=0; i<4; i++)
			feedback_array.array.push_back(*getFeedbackLED(leds[i].id, leds[i].intensity));
		feedback_pub_.publish(feedback_array);
	}

	sensor_msgs::JoyFeedbackPtr getFeedbackLED(LED led)
	{
		return getFeedbackLED(led.id, led.intensity);
	}

	/* id is 0-3 */
	sensor_msgs::JoyFeedbackPtr getFeedbackLED(ID id, INTENSITY intensity)
	{
		sensor_msgs::JoyFeedbackPtr feedback(new sensor_msgs::JoyFeedback());
		feedback->type = sensor_msgs::JoyFeedback::TYPE_LED;
		feedback->id = id;
		feedback->intensity = intensity;
		return feedback;
	}

	/* id is 0-1 */
	sensor_msgs::JoyFeedbackPtr getFeedbackRumble(ID id, INTENSITY intensity)
	{
		sensor_msgs::JoyFeedbackPtr feedback(new sensor_msgs::JoyFeedback());
		feedback->type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
		feedback->id = id;
		feedback->intensity = intensity;
		return feedback;
	}

	void knightRiderThread()
	{
		ros::WallDuration step;
		step.fromSec(0.2);

		ID id = 0;
		ID inc = +1;
		LED *leds = leds_knight_rider;

		while(ros::ok())
		{
			// at this point every leds[i] is off

			leds[id].intensity = 1;
//			sendLEDs(leds);
			updateLEDs(true);
			leds[id].intensity = 0;

			// at this point every leds[i] is off

			if(id == 0)
				inc = +1;
			else if(id == 3)
				inc = -1;

			id += inc;

			step.sleep();
		}
	}

	void updateLEDs(bool knight_rider=false)
	{
		static boost::mutex method_mutex;
		method_mutex.lock();

		bool sent_knight_rider_last_time = false;

		if (sent_knight_rider_last_time && knight_rider)
		{
			sendLEDs(leds_knight_rider);
		}
		else if (!sent_knight_rider_last_time && !knight_rider)
		{
			// only forward status leds if statii are set

			// check status leds for active leds
			bool status_set = false;
			for(int id=0; id<4; id++)
			{
				if(leds_status[id].intensity != 0)
				{
					status_set = true;
					break;
				}
			}
			if (status_set)
				sendLEDs(leds_status);
		}
		else // if both are different
		{
			// only forward knight rider leds if statii are not set

			// check status leds for active leds
			bool status_set = false;
			for(int id=0; id<4; id++)
			{
				if(leds_status[id].intensity != 0)
				{
					status_set = true;
					break;
				}
			}

			if (status_set)
			{
				if (!knight_rider)
				{
					sendLEDs(leds_status);
					sent_knight_rider_last_time = false;
				}
				else
				{
					// ignore knight rider updates when statii are set
				}
			}
			else // statii not set
			{
				sendLEDs(leds_knight_rider);
				sent_knight_rider_last_time = true;
			}
		}

		method_mutex.unlock();
	}


private:
	ros::NodeHandle nh_;
	ros::Publisher feedback_pub_;
	//ros::Subscriber bumper_sub_;
	//ros::Subscriber smach_enabled_sub_;

	LED leds_status[4];
	LED leds_knight_rider[4];

	constexpr static int LED_INDEX_MOTOR_STOP = 0;
	constexpr static int LED_INDEX_SMACH_DISABLED = 1;
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "teleop_ps3");
	ROS_INFO("Starting ps3 teleop converter, take care of your controller now...");
	//TeleopPS3 teleop_ps3;
	TeleopPS3Feedback teleop_ps3_feedback;
	ros::spin();
}
