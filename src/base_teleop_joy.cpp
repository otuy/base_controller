/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

static constexpr int LeftThumbY = 1;
static constexpr int LeftThumbX = 0;
static constexpr int RightThumbX = 2;

class BaseTeleop
{
public:
    BaseTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    double max_lin;
    double max_lin_turbo;
    double max_ang;
    double max_ang_turbo;

    static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int AxisLeftTrigger;
    static int AxisRightTrigger;
};

int BaseTeleop::ButtonA = 0;
int BaseTeleop::ButtonB = 1;
int BaseTeleop::ButtonX = 2;
int BaseTeleop::ButtonY = 3;
int BaseTeleop::ButtonLB = 4;
int BaseTeleop::ButtonRB = 5;
int BaseTeleop::ButtonSelect = 6;
int BaseTeleop::ButtonStart = 7;
int BaseTeleop::ButtonLeftThumb = 9;
int BaseTeleop::ButtonRightThumb = 10;

int BaseTeleop::AxisDPadX = 0;
int BaseTeleop::AxisDPadY = 1;
int BaseTeleop::AxisLeftThumbX = 6;
int BaseTeleop::AxisLeftThumbY = 7;
int BaseTeleop::AxisRightThumbX = 3;
int BaseTeleop::AxisRightThumbY = 4;
int BaseTeleop::AxisLeftTrigger = 2;
int BaseTeleop::AxisRightTrigger = 5;


BaseTeleop::BaseTeleop()
{

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("AxisRightThumbY", AxisRightThumbY);
    nh_.getParam("AxisLeftTriger", AxisLeftTrigger);
    nh_.getParam("AxisRightTriger", AxisRightTrigger);
    nh_.getParam("ButtonRB", ButtonRB);

    auto _nh = ros::NodeHandle("~");

    _nh.param("max_lin", this->max_lin, 1.0);
    _nh.param("max_lin_turbo", this->max_lin_turbo, this->max_lin);
    _nh.param("max_ang", this->max_ang, M_PI);
    _nh.param("max_ang_turbo", this->max_ang_turbo, this->max_ang);

    ROS_INFO("max_lin: %lf", this->max_lin);
    ROS_INFO("max_lin_turbo: %lf", this->max_lin_turbo);
    ROS_INFO("max_ang: %lf", this->max_ang);
    ROS_INFO("max_ang_turbo: %lf", this->max_ang_turbo);
}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    // double vel_x = joy->axes[AxisLeftThumbY];
    // double vel_y = joy->axes[AxisLeftThumbX];
    // double vel_z = joy->axes[AxisRightThumbX];

    double vel_x = joy->axes[AxisRightThumbY];
    double vel_y = joy->axes[AxisRightThumbX];
    double vel_z_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
    double vel_z_r = (joy->axes[AxisRightTrigger] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
    double vel_z = vel_z_l + vel_z_r;

    double vel_norm = hypot(vel_x, vel_y);
    if (vel_norm > 1.0)
    {
        vel_x /= vel_norm;
        vel_y /= vel_norm;
    }

    if (joy->buttons[ButtonRB] != 0)
    {
        vel_x *= this->max_lin_turbo;
        vel_y *= this->max_lin_turbo;
        vel_z *= this->max_ang_turbo;
    }
    else
    {
        vel_x *= this->max_lin;
        vel_y *= this->max_lin;
        vel_z *= this->max_ang;
    }

    twist.linear.x = vel_x;
    twist.linear.y = vel_y;
    twist.angular.z = vel_z;

    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_teleop_joy");

    BaseTeleop baseTeleop;

    ros::spin();
}

