/*
 * useless_planner.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>

//#include <base_controller/UselessPlannerAction.h>
//#include <actionlib/server/simple_action_server.h>

//typedef actionlib::SimpleActionServer<base_controller::UselessPlannerAction> Server;

class CubicSpline
{
private:
    double pointsPerUnit_ = 100.0;
    unsigned int skipPoints_ = 0;

public:
    // CubicSpline(){

    // }

    void interpolatePath(
        const nav_msgs::Path& path,
        nav_msgs::Path& smoothedPath)
    {
        smoothedPath.header = path.header;
        interpolatePath(path.poses, smoothedPath.poses);
    }


    void interpolatePath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& smoothedPath)
    {
        // clear new smoothed path vector in case it's not empty
        smoothedPath.clear();

        // set skipPoints_ to 0 if the path contains has too few points
        unsigned int oldSkipPoints = skipPoints_;
        skipPoints_ = std::min<int>(path.size() - 2, skipPoints_);

        // create cummulative distances vector
        std::vector<double> cummulativeDistances;
        calcCummulativeDistances(path, cummulativeDistances);

        // create temp pose
        geometry_msgs::PoseStamped pose;
        pose.header = path[0].header;

        unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);

        smoothedPath.resize(numPoints);

        ROS_INFO("numpoints:%d", numPoints);

        // interpolate points on the smoothed path using the points in the original path
        for (unsigned int i = 0; i < numPoints; i++)
        {
        double u = static_cast<double>(i) / (numPoints-1);
        interpolatePoint(path, cummulativeDistances, pose, u);

        // if (isnan(pose.pose.position.x) || isnan(pose.pose.position.y))
        //     pose.pose = smoothedPath[std::max(static_cast<int>(i)-1, 0)].pose;
        smoothedPath[i] = pose;
        }

        // copy start and goal orientations to smoothed path
        smoothedPath.front().pose.orientation = path.front().pose.orientation;
        smoothedPath.back().pose.orientation = path.back().pose.orientation;

        // revert skipPoints to original value
        skipPoints_ = oldSkipPoints;
    }

    void interpolatePoint(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        geometry_msgs::PoseStamped& point,
        double pointCummDist)
    {
        unsigned int group = findGroup(cummulativeDistances, pointCummDist);

        double a = calcAlphaCoeff(path, cummulativeDistances, group, pointCummDist);
        double b = calcBetaCoeff(path, cummulativeDistances, group, pointCummDist);
        double c = calcGammaCoeff(path, cummulativeDistances, group, pointCummDist);
        double d = calcDeltaCoeff(path, cummulativeDistances, group, pointCummDist);

        std::vector<double> grad, nextGrad;
        calcPointGradient(path, cummulativeDistances, group, grad);
        calcPointGradient(path, cummulativeDistances, group+1, nextGrad);

        point.pose.position.x =
        + a * path[group*(skipPoints_+1)].pose.position.x
        + b * path[(group+1)*(skipPoints_+1)].pose.position.x
        + c * grad[0]
        + d * nextGrad[0];

        point.pose.position.y =
        + a * path[group*(skipPoints_+1)].pose.position.y
        + b * path[(group+1)*(skipPoints_+1)].pose.position.y
        + c * grad[1]
        + d * nextGrad[1];

        double yaw;
        yaw = 
        + a * tf::getYaw(path[group*(skipPoints_+1)].pose.orientation)
        + b * tf::getYaw(path[(group+1)*(skipPoints_+1)].pose.orientation)
        + c * grad[2]
        + d * nextGrad[2];

        point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }

    void calcCummulativeDistances(
        const std::vector<geometry_msgs::PoseStamped> path,
        std::vector<double>& cummulativeDistances)
    {
        cummulativeDistances.clear();
        cummulativeDistances.push_back(0);

        for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
        cummulativeDistances.push_back(
            cummulativeDistances.back()
            + calcDistance(path, i) / calcTotalDistance(path));
    }

    double calcTotalDistance(
        const std::vector<geometry_msgs::PoseStamped>& path)
    {
        double totalDist = 0;

        for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
        totalDist += calcDistance(path, i);

        return totalDist;
    }

    double calcDistance(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx)
    {
        if (idx <= 0 || idx >=path.size())
        return 0;

        double dist =
        hypot(
            path[idx].pose.position.x - path[idx-skipPoints_-1].pose.position.x,
            path[idx].pose.position.y - path[idx-skipPoints_-1].pose.position.y);

        return dist;
    }

    double calcAlphaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
    {
        double alpha =
        + 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
        - 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)
        + 1;
        return alpha;
    }

    double calcBetaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
    {
        double beta =
        - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
        + 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2);
        return beta;
    }

    double calcGammaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
    {
        double gamma =
        (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
        - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
        * (cummulativeDistances[idx+1] - cummulativeDistances[idx])
        + input
        - cummulativeDistances[idx];
        return gamma;
    }

    double calcDeltaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input)
    {
        double delta =
        (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
        - pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
        * (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
        return delta;
    }

    double calcRelativeDistance(
        const std::vector<double>& cummulativeDistances,
        const unsigned int idx,
        const double input)
    {
        double relDist =
        (input - cummulativeDistances[idx])
        / (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
        return relDist;
    }


    void calcPointGradient(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        unsigned int idx,
        std::vector<double>& gradient)
    {
        double dx, dy, dth, du;
        gradient.assign(3, 0);

        if (idx == 0 || idx == cummulativeDistances.size()-1)
        return;

        dx = path[(idx)*(skipPoints_+1)].pose.position.x - path[(idx-1)*(skipPoints_+1)].pose.position.x;
        dy = path[(idx)*(skipPoints_+1)].pose.position.y - path[(idx-1)*(skipPoints_+1)].pose.position.y;
        dth = tf::getYaw(path[(idx)*(skipPoints_+1)].pose.orientation) - tf::getYaw(path[(idx-1)*(skipPoints_+1)].pose.orientation);
        du = cummulativeDistances[idx] - cummulativeDistances[idx-1];

        if (dth >= M_PI)
        {
        dth -= (2 * M_PI);
        }
        else if (dth < -M_PI)
        {
        dth += (2 * M_PI);
        }

        gradient[0] = dx / du;
        gradient[1] = dy / du;
        gradient[2] = dth / du;
    }


    unsigned int findGroup(
        const std::vector<double>& cummulativeDistances,
        double pointCummDist)
    {
        unsigned int i;
        for (i = 0; i < cummulativeDistances.size()-1; i++)
        {
            if (pointCummDist <= cummulativeDistances[i+1])
                return i;
        }
        return i;
    }
};

class UselessPlanner
{
public:
    UselessPlanner(std::string name);

    //void SetMaximumAcceleration(double acc);
    //void SetMaximumVelocity(double vel);
    //void execute_action(const base_controller::UselessPlannerActionGoalConstPtr& goal, Server* as);

private:
    void PathCallback(const nav_msgs::Path::ConstPtr& msg);
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void AbortCallback(const std_msgs::Bool::ConstPtr& msg);
    void LinToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
    void AngToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
    void TimerCallback(const ros::TimerEvent& event);
    void ManualCallback(const std_msgs::Bool::ConstPtr& msg);
    //void CalcWheelSpeed(double actualDt);

    inline double getYawFromQuat(const geometry_msgs::Quaternion& quat)
    {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return tf::getYaw(q);
    }

    geometry_msgs::Pose2D pose_to_2d(geometry_msgs::Pose pose)
    {
        geometry_msgs::Pose2D r;
        r.x = pose.position.x;
        r.y = pose.position.y;
        r.theta = getYawFromQuat(pose.orientation);

        return r;
    }

    ros::NodeHandle nh_;
    //actionlib::SimpleActionServer<base_controller::UselessPlannerAction> as_;
    std::string action_name_;
    //base_controller::UselessPlannerActionFeedback feedback_;
    //base_controller::UselessPlannerActionResult result_;

    double vel_lim_lin;
    double vel_lim_ang;

    double accel_lim_lin;
    double accel_lim_ang;
    double deccel_lim_lin;
    double deccel_lim_ang;

    double jerk_lim_lin;
    double jerk_lim_ang;

    double base_vel_k_tan;
    double base_vel_k_nor;
    double base_vel_k_ang;

    double look_ahead_distance;

    double lin_goal_tolerance;
    double ang_goal_tolerance;

    double lin_waypoint_tolerance;

    double control_interval = 1 / 20.0;

    bool planning = false;
    bool _is_manual_enabled = false;

    //double MaximumVelocity;

    ros::Subscriber path_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber abort_sub;
    ros::Subscriber manual_sub;

    //ros::Subscriber lin_goal_tolerance_sub;
    //ros::Subscriber ang_goal_tolerance_sub;

    //ros::Subscriber lin_waypoint_tolerance_sub;
    //ros::Subscriber ang_waypoint_tolerance_sub;

    ros::Publisher goal_reached_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher fine_path_pub;
    //ros::Publisher path_pub;
    ros::Timer control_tim;

    tf::TransformListener _tflistener;

    //double targetVelX;
    //double targetVelY;
    //double targetRotZ;

    //double targetTime;

    //double lastTarget[3];// = {0.0, 0.0, 0.0};
    //std_msgs::Int16MultiArray motorCmdVel_msg;
    std_msgs::Bool goal_reached_msg;
    geometry_msgs::Twist cmd_vel_msg;

    geometry_msgs::Pose2D last_target_msg;
    geometry_msgs::Pose2D last_pose_msg;

    //nav_msgs::Path path_msg;
    nav_msgs::Path fine_target_path;
    nav_msgs::Path target_path;
    int current_goal_index = 0;
    int current_waypoint_index = 0;
};

UselessPlanner::UselessPlanner(std::string name)// :
//        as_(nh_, name, false), action_name_(name)
{
    auto private_nh = ros::NodeHandle("~/UselessPlanner");
    //this->base_max_vel_lin = 1.0;
    //this->base_max_vel_ang = 3.0;

    //this->base_vel_k_lin = 1.0;
    //this->base_vel_k_ang = 1.0;

    //this->lin_goal_tolerance = 0.03;
    //this->ang_goal_tolerance = 0.02;

    private_nh.param("vel_lim_lin", this->vel_lim_lin, 1.0);
    private_nh.param("vel_lim_ang", this->vel_lim_ang, 3.0);

    ROS_INFO("vel_lim_lin : %f", this->vel_lim_lin);
    ROS_INFO("vel_lim_ang : %f", this->vel_lim_ang);

    private_nh.param("acc_lim_lin", this->accel_lim_lin, 1.0);
    private_nh.param("acc_lim_ang", this->accel_lim_ang, 1.0);

    ROS_INFO("acc_lim_lin : %f", this->accel_lim_lin);
    ROS_INFO("acc_lim_ang : %f", this->accel_lim_ang);

    // defaults to the same value as acc_lim
    private_nh.param("dec_lim_lin", this->deccel_lim_lin, this->accel_lim_lin);
    private_nh.param("dec_lim_ang", this->deccel_lim_ang, this->accel_lim_ang);

    private_nh.param("jerk_lim_lin", this->jerk_lim_lin, 1.0);
    private_nh.param("jerk_lim_ang", this->jerk_lim_ang, 1.0);

    private_nh.param("base_vel_k_tan", this->base_vel_k_tan, 1.0);
    private_nh.param("base_vel_k_nor", this->base_vel_k_nor, 2.0);
    private_nh.param("base_vel_k_ang", this->base_vel_k_ang, 0.5);

    ROS_INFO("base_vel_k_tan : %f", this->base_vel_k_tan);
    ROS_INFO("base_vel_k_nor : %f", this->base_vel_k_nor);
    ROS_INFO("base_vel_k_ang : %f", this->base_vel_k_ang);

    private_nh.param("goal_tolerance_lin", this->lin_goal_tolerance, 0.05);					// 5 centimetre
    private_nh.param("goal_tolerance_ang", this->ang_goal_tolerance, (M_PI * 5.0 / 180.0));	// \pm 5 deg
    private_nh.param("wp_tolerance_lin", this->lin_waypoint_tolerance, 0.50);			// 50 centimetre

    ROS_INFO("lin_goal_tolerance : %f [m]", this->lin_goal_tolerance);
    ROS_INFO("ang_goal_tolerance : %f [m]", this->ang_goal_tolerance);
    ROS_INFO("lin_waypoint_tolerance : %f [m]", this->lin_waypoint_tolerance);

    look_ahead_distance = pow(vel_lim_lin + (accel_lim_lin * control_interval), 2) / (2.0 * accel_lim_lin);
    ROS_INFO("look-ahead distance : %f [m]", look_ahead_distance);

    path_sub = nh_.subscribe<nav_msgs::Path>("target_path", 10, &UselessPlanner::PathCallback, this);
    //odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &UselessPlanner::PoseCallback, this);

    abort_sub = nh_.subscribe<std_msgs::Bool>("abort", 10, &UselessPlanner::AbortCallback, this);
    manual_sub = nh_.subscribe<std_msgs::Bool>("manual", 10, &UselessPlanner::ManualCallback, this);

    //lin_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("lin_tolerance", 10, &UselessPlanner::LinToleranceCallback, this);
    //ang_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("ang_tolerance", 10, &UselessPlanner::AngToleranceCallback, this);

    goal_reached_pub = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);
    fine_path_pub = nh_.advertise<nav_msgs::Path>("fine_target_path", 1, true);

    control_tim = nh_.createTimer(ros::Duration(control_interval), &UselessPlanner::TimerCallback, this);

    //Server server(uselessPlanner->nh, "do_dishes", boost::bind(&UselessPlanner::execute_action, uselessPlanner, _1, &server), false);
    //server.start();
}

void UselessPlanner::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    // TODO : generate finer path for better path tracking characteristics

    this->fine_target_path.poses.clear();

    // std::vector<geometry_msgs::PoseStamped> poses(msg->poses);
    // std::vector<geometry_msgs::PoseStamped> fine_poses;

    // poses.insert(poses.begin(), 2, poses.front());
    // poses.push_back(poses.back());
    // poses.push_back(poses.back());

    this->target_path = *msg;

    ROS_INFO("received path msg.");

    ROS_INFO("path has %ld poses", target_path.poses.size());

    //std::vector<BSplineSegment> segments;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    CubicSpline cubicspline;
    cubicspline.interpolatePath(target_path, fine_target_path);

    double totalLength = 0.0;
    /*
    for(auto it = fine_poses.begin() + 1; it < fine_poses.end(); it++)
    {
        double delta_x = (it->pose.position.x - (it - 1)->pose.position.x);
        double delta_y = (it->pose.position.y - (it - 1)->pose.position.y);
        double l = hypot(delta_x, delta_y);
        totalLength += l;
    }
     */

#if 0
    for (auto seg : segments)
    {
        totalLength += seg.getLength();
    }

    // totalLength exceeding decisionDelta results in a trapezoidal motion profile
    double decisionDelta = pow(vel_lim_lin, 2) / accel_lim_lin;

    int accel_until_ts = 0.0;
    int cruise_until_ts = 0.0;
    int decel_until_ts = 0.0;
    cruise_until_ts = (totalLength - decisionDelta) / vel_lim_lin / control_interval;

    if (cruise_until_ts > 0)
    {
        //trapezoidal profile
        accel_until_ts = vel_lim_lin / accel_lim_lin / control_interval;
        //decel_until_m = cruise_until_m + (decisionDelta / 2.0);
        decel_until_ts = vel_lim_lin / accel_lim_lin / control_interval;
    }
    else
    {
        // triangle profile
        ROS_ERROR("triangle profile is not supported.");
        throw new std::exception();
    }

    // sample
    double currentVelocity = 0.0;
    double traveledTotal = 0.0;
    int current_ts = 0;

    std::vector<geometry_msgs::PoseStamped> filtered_poses;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    double currentSegmentTraveled = 0.0;

    for (auto segment : segments)
    {
        double segmentLength = segment.getLength();

        while (currentSegmentTraveled < segmentLength)
        {
            ROS_INFO("%lf of %lf", currentSegmentTraveled, segmentLength);
            segment.getPointFromDistance(currentSegmentTraveled, pose.pose.position.x, pose.pose.position.y);
            filtered_poses.push_back(pose);

            currentSegmentTraveled += currentVelocity * control_interval;
            traveledTotal += currentVelocity * control_interval;

            if (current_ts < accel_until_ts)
            {
                currentVelocity += accel_lim_lin * control_interval;
            }
            else if (current_ts < cruise_until_ts)
            {
                currentVelocity = vel_lim_lin;
            }
            else if (current_ts < decel_until_ts)
            {
                currentVelocity -= accel_lim_lin * control_interval;
            }
            else
            {
                currentVelocity = 0.0;    //accel_lim_lin * control_interval;
                break;
            }

            ROS_INFO("current velocity: %lf", currentVelocity);

            current_ts++;
        }

        //currentSegmentTraveled = segmentLength - currentSegmentTraveled;
    }
#endif

    //this->target_path = *msg;
    this->current_goal_index = 0;
    this->current_waypoint_index = 0;

    if (fine_target_path.poses.size() <= 0)
    {
        return;
    }

    this->fine_target_path.header.frame_id = "map";
    this->fine_target_path.header.stamp = ros::Time::now();
    // this->fine_target_path.poses = fine_poses;

    this->fine_path_pub.publish(this->fine_target_path);
    ROS_INFO("fine path has %ld poses", fine_target_path.poses.size());

    this->planning = true;
}

void UselessPlanner::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double yaw = getYawFromQuat(msg->pose.pose.orientation);

    this->last_pose_msg.x = msg->pose.pose.position.x;
    this->last_pose_msg.y = msg->pose.pose.position.y;
    this->last_pose_msg.theta = yaw;
}

void UselessPlanner::AbortCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        this->planning = false;
        this->last_target_msg.x = this->last_pose_msg.x;
        this->last_target_msg.y = this->last_pose_msg.y;
        this->last_target_msg.theta = this->last_pose_msg.theta;

        this->cmd_vel_msg.linear.x = 0;
        this->cmd_vel_msg.linear.y = 0;
        this->cmd_vel_msg.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel_msg);
    }
}

void UselessPlanner::ManualCallback(const std_msgs::Bool::ConstPtr& msg)
{
    this->_is_manual_enabled = msg->data;
}

void UselessPlanner::TimerCallback(const ros::TimerEvent& event)
{
    if(this->_is_manual_enabled)
    {
        return;
    }

    if (fine_target_path.poses.size() <= 0l)
    {
        //ROS_INFO("paths size: %ld", fine_target_path.poses.size());
        this->planning = false;
        current_goal_index = 0;
    }

    if (!this->planning)
    {
        this->cmd_vel_msg.linear.x = 0;
        this->cmd_vel_msg.linear.y = 0;
        this->cmd_vel_msg.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel_msg);

        return;
    }

    geometry_msgs::Pose2D target_pose, goal_pose;
    double vel_norm;
    static double last_vel_norm = 0.0;
    static double last_vel_z = 0.0;
    double vel_z;
    //double vel_t, vel_r, theta;
    //double pose_delta_world_x;
    //double pose_delta_world_y;

    double target_angle;
    //double vel_tan, vel_nor;
    //double diff_tan, diff_nor;

    double dt = event.current_real.toSec() - event.last_real.toSec();
    double acc_max = this->accel_lim_lin * dt / 1.5;

    tf::StampedTransform base_link;

    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        ROS_INFO("failed to look up tf.");
        this->cmd_vel_msg.linear.x = 0;
        this->cmd_vel_msg.linear.y = 0;
        this->cmd_vel_msg.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel_msg);

        return;
    }

    this->last_pose_msg.x = base_link.getOrigin().x();
    this->last_pose_msg.y = base_link.getOrigin().y();
    this->last_pose_msg.theta = tf::getYaw(base_link.getRotation());

    goal_pose = pose_to_2d(this->fine_target_path.poses.back().pose);

    while (1)
    {
        // 到達判定

        target_pose = pose_to_2d(this->fine_target_path.poses.at(this->current_waypoint_index).pose);

        double vel_world_x = target_pose.x - this->last_pose_msg.x;
        double vel_world_y = target_pose.y - this->last_pose_msg.y;

        vel_z = target_pose.theta - this->last_pose_msg.theta;
        //vel_z = goal_pose.theta - this->last_pose_msg.theta;

        target_angle = atan2(vel_world_y, vel_world_x);
        vel_norm = hypot(vel_world_x, vel_world_y);

        // 旋回方向最適化
        if (vel_z > M_PI)
        {
            vel_z -= (2 * M_PI);
        }
        else if (vel_z < -M_PI)
        {
            vel_z += (2 * M_PI);
        }

        if (this->current_waypoint_index < (this->fine_target_path.poses.size() - 1))
        {
            // current target is a waypoint, not the goal

            if (vel_norm < this->look_ahead_distance)
            {
                // waypoint reached
                this->current_waypoint_index++;
                continue;
            }
            else
            {
                // waypoint, still tracking
                vel_norm = vel_lim_lin;
                vel_z *= base_vel_k_ang;
            }
        }
        else if (this->current_waypoint_index == (this->fine_target_path.poses.size() - 1))
        {
            // current target is the goal, not a waypoint.
            // final approach
            if ((vel_norm < this->lin_goal_tolerance) && (fabs(vel_z) < this->ang_goal_tolerance)
                    && (hypot(this->cmd_vel_msg.linear.x, this->cmd_vel_msg.linear.y) < this->accel_lim_lin * dt)
                    && (fabs(this->cmd_vel_msg.angular.z) < this->accel_lim_ang * dt))
            {
                // goal reached

                this->planning = false;

                this->cmd_vel_msg.linear.x = 0.0;
                this->cmd_vel_msg.linear.y = 0.0;
                this->cmd_vel_msg.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel_msg);

                this->goal_reached_msg.data = true;
                this->goal_reached_pub.publish(this->goal_reached_msg);
                this->goal_reached_msg.data = false;

                return;
            }
            else
            {
                // goal, final approach

            }
        }

        this->goal_reached_msg.data = false;
        //this->goal_reached_pub.publish(this->goal_reached_msg);
        break;
    }

    while (1)
    {
        // 到達判定
        auto _target_pose = pose_to_2d(this->fine_target_path.poses.at(this->current_goal_index).pose);

        double vel_world_x = _target_pose.x - this->last_pose_msg.x;
        double vel_world_y = _target_pose.y - this->last_pose_msg.y;

        double _vel_z = _target_pose.theta - this->last_pose_msg.theta;

        double _target_angle = atan2(vel_world_y, vel_world_x);
        double _vel_norm = hypot(vel_world_x, vel_world_y);

        if (this->current_goal_index < (this->fine_target_path.poses.size() - 1))
        {
            // current target is a waypoint, not the goal

            if (_vel_norm < this->look_ahead_distance)
            {
                // waypoint reached
                this->current_goal_index++;
                continue;
            }
            else
            {
                // waypoint, still tracking

            }
        }
        else if (this->current_goal_index == (this->fine_target_path.poses.size() - 1))
        {
            // current target is the goal, not a waypoint.
            // final approach
            if ((_vel_norm < this->lin_goal_tolerance) && (fabs(_vel_z) < this->ang_goal_tolerance)
                    && (hypot(this->cmd_vel_msg.linear.x, this->cmd_vel_msg.linear.y) < this->accel_lim_lin * dt)
                    && (fabs(this->cmd_vel_msg.angular.z) < this->accel_lim_ang * dt))
            {
                // goal reached

                this->planning = false;

                this->cmd_vel_msg.linear.x = 0.0;
                this->cmd_vel_msg.linear.y = 0.0;
                this->cmd_vel_msg.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel_msg);

                this->goal_reached_msg.data = true;
                this->goal_reached_pub.publish(this->goal_reached_msg);
                this->goal_reached_msg.data = false;

                return;
            }
            else
            {
                // goal, final approach

                target_pose = _target_pose;
                //vel_z = _vel_z;
                target_angle = _target_angle;

                double v_sq = (1.0 * accel_lim_lin * (_vel_norm - (last_vel_norm * control_interval)));
                if (v_sq < 0) v_sq = 0;
                vel_norm = sqrt(v_sq);

                if (this->ang_goal_tolerance < fabs(_vel_z))
                {
                    v_sq = (1.0 * accel_lim_ang * fabs(_vel_z - (last_vel_z * control_interval)));
                    if (v_sq < 1e-12)
                    {
                        vel_z = 0.0;
                    }
                    else
                    {
                        vel_z = sqrt(v_sq) * (_vel_z / fabs(_vel_z));
                    }
                }
                else
                {
                    vel_z = 0.0;
                }

                //vel_norm = this->base_vel_k_tan * _vel_norm;

            }
        }

        break;
    }

    bool tracking_goal = (this->current_goal_index == (this->fine_target_path.poses.size() - 1));

    double lim_half_value_diff_rot = 1.5;
    if(fabs(vel_z) > lim_half_value_diff_rot / 2.0)
    {
        vel_norm *= lim_half_value_diff_rot / (2 * fabs(vel_z));
    }

    //double vel = sqrt(2.0 * accel_lim_lin * vel_norm * 0.9);
    //vel = sqrt(2.0 * accel_lim_lin * (vel_norm - (vel * control_interval)));
    //if(vel > vel_lim_lin)
    //{
    //    vel = vel_lim_lin;
    //}

    //vel_tan = vel * cos(angle_diff);
    //vel_nor = vel * sin(angle_diff);
    //vel_tan = sqrt(2.0 * accel_lim_lin * diff_tan);
    //vel_nor = sqrt(2.0 * accel_lim_lin * diff_nor);

#if 0
    /double vel_z_norm = fabs(vel_z);
    if(vel_z_norm > 0.0000001)
    {
        double _r = fabs(sqrt(2 * this->accel_lim_ang * vel_z_norm) - (this->accel_lim_ang * dt)) / vel_z_norm;
        vel_z *= _r;
    }
#endif

    // limit angular velocity
    double vel_z_norm = fabs(vel_z);
    if (vel_z_norm > this->vel_lim_ang)
    {
        double _r = this->vel_lim_ang / vel_z_norm;
        vel_z *= _r;
    }

    //pose_delta_world_x -= this->cmd_vel_msg.linear.x * dt;
    //pose_delta_world_y -= this->cmd_vel_msg.linear.y * dt;

    // angle from origin
    //theta = this->last_pose_msg.theta - atan2(-pose_delta_world_x, pose_delta_world_y);
    //theta += this->cmd_vel_msg.angular.z * dt;

    //double theta = this->last_pose_msg.theta - path_angle;
    //double tmp_x = +(vel_tan * cos(theta)) + (vel_nor * sin(theta));
    //double tmp_y = -(vel_tan * sin(theta)) + (vel_nor * cos(theta));

    double theta = this->last_pose_msg.theta;
    double tmp_x = +(vel_norm * cos(target_angle) * cos(theta)) + (vel_norm * sin(target_angle) * sin(theta));
    double tmp_y = -(vel_norm * cos(target_angle) * sin(theta)) + (vel_norm * sin(target_angle) * cos(theta));

    double vel_x;
    double vel_y;
    vel_x = tmp_x;
    vel_y = tmp_y;

    vel_norm = hypot(vel_x, vel_y);

    if (vel_norm > this->vel_lim_lin)
    {
        double _r = this->vel_lim_lin / vel_norm;
        vel_x *= _r;
        vel_y *= _r;
    }

    /*
    double vel_norm = hypot(vel_world_x, vel_world_y);
    if(vel_norm > this->vel_lim_lin)
    {
        double r = this->vel_lim_lin / vel_norm;
        vel_world_x *= r;
        vel_world_y *= r;
    }

    double yaw = this->last_pose_msg.theta;
    double vel_x = (vel_world_x * cos(-yaw)) - (vel_world_y * sin(-yaw));
    double vel_y = (vel_world_x * sin(-yaw)) + (vel_world_y * cos(-yaw));
     */
    // accel. limit
#define ACCEL_LIMIT
#ifdef ACCEL_LIMIT
    double acc_x = vel_x - cmd_vel_msg.linear.x;
    double acc_y = vel_y - cmd_vel_msg.linear.y;
    double acc_trans_norm = hypot(acc_x, acc_y);
    if (acc_trans_norm > accel_lim_lin * dt)
    {
        double _r = accel_lim_lin * dt / acc_trans_norm;
        acc_x *= _r;
        acc_y *= _r;
    }

    // TODO: JERK limit

    this->cmd_vel_msg.linear.x += acc_x;
    this->cmd_vel_msg.linear.y += acc_y;
#else
    this->cmd_vel_msg.linear.x = vel_x;
    this->cmd_vel_msg.linear.y = vel_y;
#endif

    last_vel_norm = hypot(this->cmd_vel_msg.linear.x, this->cmd_vel_msg.linear.y);

    this->cmd_vel_msg.angular.z = vel_z;

    last_vel_z = vel_z;

    cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "useless_planner");

    UselessPlanner *uselessPlanner = new UselessPlanner(ros::this_node::getName());
    ROS_INFO("cubic_spline_planner node has started.");

    ros::spin();
    ROS_INFO("cubic_spline_planner node has been terminated.");
}