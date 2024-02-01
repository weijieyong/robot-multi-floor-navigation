#include "robot-multi-floor-navigation/multi_floor_navigation.h"


MultiFloorNavigation::MultiFloorNavigation() :
    state(MultiFloorNavigation::States::INIT_POSE),
    is_goal_sent(false),
    is_goal_active(false)
{
}

MultiFloorNavigation::~MultiFloorNavigation()
{
}

void MultiFloorNavigation::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    
    amcl_pose_sub = n.subscribe("amcl_pose", 1, &MultiFloorNavigation::amclPoseCallback, this);
    move_base_status_sub = n.subscribe("move_base/status", 1, &MultiFloorNavigation::movebaseStatusCallback, this);

}

void MultiFloorNavigation::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    curr_pose.pose = msg->pose;
}

void MultiFloorNavigation::movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    move_base_status_msg = *msg;
}

tf2::Quaternion MultiFloorNavigation::yawtoQuartenion(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    return q;
}

void MultiFloorNavigation::initializeRobotPose(double x, double y, double z, double yaw){
    geometry_msgs::PoseWithCovarianceStamped initial_pose;

    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = ros::Time::now();

    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = z;

    tf2::Quaternion q = yawtoQuartenion(yaw);

    initial_pose.pose.pose.orientation.x= q[0];
    initial_pose.pose.pose.orientation.y= q[1];
    initial_pose.pose.pose.orientation.z= q[2];
    initial_pose.pose.pose.orientation.w= q[3];

    ROS_INFO("Initializing robot at x: %.1f, y: %.1f, z: %.1f, yaw: %.1f", x, y, z, yaw);
    initial_pose_pub.publish(initial_pose);
}

void MultiFloorNavigation::sendSimpleGoal(double x, double y, double z, double yaw)
{
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;

    tf2::Quaternion q = yawtoQuartenion(yaw);

    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    ROS_INFO("Sending robot to x: %.1f, y: %.1f, z: %.1f, yaw: %.1f", x, y, z, yaw);
    goal_pub.publish(goal);
}

void MultiFloorNavigation::execute()
{
    switch(state){
        case MultiFloorNavigation::States::INIT_POSE:
            ROS_INFO_ONCE("Initializing robot at x:4.0, y:-5.0, z:0.5, yaw:0.0deg");
            initializeRobotPose(4.0, -5.0, 0.5, 0.0); // TODO: hard-coded
            state = MultiFloorNavigation::States::NAV_TO_GOAL;
            break;

        case MultiFloorNavigation::States::NAV_TO_GOAL:
            ROS_INFO_ONCE("Sending goal1");
            if(!is_goal_sent) {
                sendSimpleGoal(3.0, -0.5, 0.5, M_PI);
                is_goal_sent = true;
            }
            if(!move_base_status_msg.status_list.empty()) {
                auto& currentStatus = move_base_status_msg.status_list[0].status;

                if(currentStatus == actionlib_msgs::GoalStatus::ACTIVE) {
                    is_goal_active = true;
                }
                else if(is_goal_active && 
                    currentStatus == actionlib_msgs::GoalStatus::SUCCEEDED) {
                    state = MultiFloorNavigation::States::DONE;
                    is_goal_sent = is_goal_active = false;
                }
            }
            break;

        case MultiFloorNavigation::States::DONE:
            ROS_INFO_ONCE("Robot reached goal");
            break;

        default:
            ROS_ERROR("SOME ERROR");
    }

}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "multi_floor_navigation_node");
    ros::NodeHandle nh;

    MultiFloorNavigation multi_floor_nav;
    multi_floor_nav.initialize(nh);

    ros::Rate rate(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        multi_floor_nav.execute();
        rate.sleep();
    }

    return 0;
}