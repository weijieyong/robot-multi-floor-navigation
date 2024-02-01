#pragma once

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class MultiFloorNavigation
{
private:
    enum States{
        INIT_POSE,
        DONE
    };
    States state;
    ros::Publisher initial_pose_pub, goal_pub;
    ros::Subscriber amcl_pose_sub;
    geometry_msgs::PoseWithCovarianceStamped curr_pose;

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void initializeRobotPose(double x, double y, double z, double yaw);
    tf2::Quaternion yawtoQuartenion(double yaw);

public:
    MultiFloorNavigation();
    ~MultiFloorNavigation();
    void initialize(ros::NodeHandle& n);
    void execute();

};
