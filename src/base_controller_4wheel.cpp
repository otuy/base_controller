/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <math.h>

class BaseController
{
public:
	BaseController();

private:
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	void CalcWheelSpeed(double actualDt);

	double MaximumAcceleration;
	double MaximumVelocity;
	double RobotRadius;
	double wheel_radius;

	bool InvertX = false;
	bool InvertY = false;
	bool InvertZ = false;

	bool LimitVelocity = true;
	bool LimitAcceleration = true;

	ros::NodeHandle nh;

	ros::Subscriber cmdVel_sub;
	ros::Timer control_tim;

	ros::Publisher motor0CmdVel_pub;
	ros::Publisher motor1CmdVel_pub;
	ros::Publisher motor2CmdVel_pub;
    ros::Publisher motor3CmdVel_pub;

	double targetVelX;
	double targetVelY;
	double targetRotZ;

	ros::Time targetTime;

	double lastTarget[4];
	std_msgs::Float64 motorCmdVelmsg[4];
};

BaseController::BaseController(void)
{
	auto _nh = ros::NodeHandle("~");

	_nh.param("motor_max_acc", this->MaximumAcceleration, 0.0);
	_nh.param("motor_max_vel", this->MaximumVelocity, 0.0);
	_nh.param("robot_radius", this->RobotRadius, 0.258);
	_nh.param("wheel_radius", this->wheel_radius, 0.0500);

	ROS_INFO("motor_max_acc : %f", this->MaximumAcceleration);
	ROS_INFO("motor_max_vel : %f", this->MaximumVelocity);
	ROS_INFO("robot_radius : %f", this->RobotRadius);

	if(this->MaximumVelocity < 0)
	{
		this->LimitVelocity = false;
	}

	if(this->MaximumAcceleration < 0)
	{
		this->LimitAcceleration = false;
	}

	_nh.param("invert_x", this->InvertX, false);
	_nh.param("invert_y", this->InvertY, false);
	_nh.param("invert_z", this->InvertZ, false);

	int ctrl_freq;
	_nh.param("ctrl_freq", ctrl_freq, 20);

	cmdVel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &BaseController::CmdVelCallback, this);

	motor0CmdVel_pub = nh.advertise<std_msgs::Float64>("motor0_cmd_vel", 1);
	motor1CmdVel_pub = nh.advertise<std_msgs::Float64>("motor1_cmd_vel", 1);
	motor2CmdVel_pub = nh.advertise<std_msgs::Float64>("motor2_cmd_vel", 1);
    motor3CmdVel_pub = nh.advertise<std_msgs::Float64>("motor3_cmd_vel", 1);

	control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &BaseController::TimerCallback, this);

	targetVelX = targetVelY = targetRotZ = 0.0;

	lastTarget[0] = 0.0;
	lastTarget[1] = 0.0;
	lastTarget[2] = 0.0;
    lastTarget[3] = 0.0;
	motorCmdVelmsg[0].data = 0.0;
	motorCmdVelmsg[1].data = 0.0;
	motorCmdVelmsg[2].data = 0.0;
    motorCmdVelmsg[3].data = 0.0;
}

void BaseController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	this->targetVelX = static_cast<double>(msg->linear.x);
	this->targetVelY = static_cast<double>(msg->linear.y);
	this->targetRotZ = static_cast<double>(msg->angular.z);
	this->targetTime = ros::Time::now();

	if(this->InvertX)
	{
		this->targetVelX *= -1;
	}
	if(this->InvertY)
	{
		this->targetVelY *= -1;
	}
	if(this->InvertZ)
	{
		this->targetRotZ *= -1;
	}
}

void BaseController::TimerCallback(const ros::TimerEvent& event)
{
	CalcWheelSpeed(event.current_real.toSec() - event.last_real.toSec());
	motor0CmdVel_pub.publish(motorCmdVelmsg[0]);
	motor1CmdVel_pub.publish(motorCmdVelmsg[1]);
	motor2CmdVel_pub.publish(motorCmdVelmsg[2]);
    motor3CmdVel_pub.publish(motorCmdVelmsg[3]);
}

void BaseController::CalcWheelSpeed(double actualDt){
	double t[4];

	t[0] = -(									  (targetVelY * 1)				    + (targetRotZ * RobotRadius)) / wheel_radius;
	t[1] = -((targetVelX * sin(1 * M_PI / 2))	+ (targetVelY * cos(1 * M_PI / 2)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
	t[2] = -((targetVelX * sin(2 * M_PI / 2))	+ (targetVelY * cos(2 * M_PI / 2)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;
    t[3] = -((targetVelX * sin(3 * M_PI / 2))	+ (targetVelY * cos(3 * M_PI / 2)) 	+ (targetRotZ * RobotRadius)) / wheel_radius;

	double _k = 1.0;

	if(this->LimitVelocity){
		for(int i = 0; i < 4; i++){
			auto _a = fabs(t[i]);
			if(_a * _k > this->MaximumVelocity){
				_k = this->MaximumVelocity / _a;
                ROS_WARN("An infeasible velocity command detected! You might want to look into it.");
			}
		}

		for(int i = 0; i < 4; i++){
			t[i] *= _k;
		}
	}

	if(this->LimitAcceleration){
		float maxVelDelta = this->MaximumAcceleration * actualDt;

		_k = 1.0;

		for(int i = 0; i < 4; i++){
			double diffabs = fabs(t[i] - lastTarget[i]);
			if(diffabs * _k > maxVelDelta){
				_k = maxVelDelta / diffabs;
				ROS_WARN("An infeasible acceleration detected! You might want to look into it.");
			}
		}

		for(int i = 0; i < 4; i++){
			t[i] = lastTarget[i] + ((t[i] - lastTarget[i]) * _k);
		}
	}

	for(int i = 0; i < 4; i++){
		this->lastTarget[i] = t[i];
		this->motorCmdVelmsg[i].data = t[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_controller");

	BaseController *baseController = new BaseController();
	ROS_INFO("base_controller node has started.");

	ros::spin();
	ROS_INFO("base_controller node has been terminated.");
}

