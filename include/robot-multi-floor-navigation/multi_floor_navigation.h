#pragma once

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "actionlib_msgs/GoalStatusArray.h"


class MultiFloorNavigation
{
private:
    enum States{
        INIT_POSE,
        NAV_TO_GOAL,
        DONE
    };
    States state;
    ros::Publisher initial_pose_pub, goal_pub;
    ros::Subscriber amcl_pose_sub, move_base_status_sub;
    geometry_msgs::PoseWithCovarianceStamped curr_pose;
    actionlib_msgs::GoalStatusArray move_base_status_msg;
    bool is_goal_sent, is_goal_active;

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void initializeRobotPose(double x, double y, double z, double yaw);
    tf2::Quaternion yawtoQuartenion(double yaw);
    void sendSimpleGoal(double x, double y, double z, double yaw);

public:
    MultiFloorNavigation();
    ~MultiFloorNavigation();
    void initialize(ros::NodeHandle& n);
    void execute();

};
