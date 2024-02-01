#include "multi_floor_navigation/multi_floor_navigation.h"


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
    elevator_pub = n.advertise<std_msgs::String>("elevator", 1, true);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    
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

    initial_pose.pose.pose.orientation.x = q[0];
    initial_pose.pose.pose.orientation.y = q[1];
    initial_pose.pose.pose.orientation.z = q[2];
    initial_pose.pose.pose.orientation.w = q[3];

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

void MultiFloorNavigation::callElevatorFloor(std::string floor)
{
    std_msgs::String floor_no;
    floor_no.data = floor;
    elevator_pub.publish(floor_no);
}

void MultiFloorNavigation::publishCmdVel(double linear_vel=0.0, double angular_vel=0.0){
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear_vel;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = angular_vel;

    cmd_vel_pub.publish(cmd_vel);
}

void MultiFloorNavigation::execute()
{
    switch(state){
        case States::INIT_POSE:
            ROS_INFO_ONCE("Initializing robot at x:4.0, y:-5.0, z:0.5, yaw:0.0deg");
            initializeRobotPose(4.0, -5.0, 0.5, 0.0); // TODO: hard-coded
            state = States::NAV_TO_GOAL;
            break;

        case States::NAV_TO_GOAL:
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
                    state = States::CALL_ELEVATOR_0;
                    is_goal_sent = is_goal_active = false;
                }
            }
            break;

        case States::CALL_ELEVATOR_0:
            ROS_INFO_ONCE("Calling elevator at Floor 0");
            callElevatorFloor("0");
            state = States::ENTER_ELEVATOR_0;
            break;

        case States::ENTER_ELEVATOR_0:
            ROS_INFO_ONCE("Entering elevator at Floor 0");
            
            if(fabs(curr_pose.pose.pose.position.x - 0.5) < 0.1) { // TODO: hard coded tolerance
                publishCmdVel(0.0, 0.0);
                ros::Duration(7).sleep(); // wait for elevator to close
                state = States::CALL_ELEVATOR_1;
            }
            else {
                publishCmdVel(0.3, 0.0);
            }
            break;

        case States::CALL_ELEVATOR_1:
            ROS_INFO_ONCE("Calling elevator to Floor 1");
            callElevatorFloor("1");
            ros::Duration(7).sleep(); // wait for elevator to travel to level 1 and open
            state = States::EXIT_ELEVATOR_1;
            break;

        case States::EXIT_ELEVATOR_1:
            ROS_INFO_ONCE("Exiting elevator at Floor 1");
            
            if(fabs(curr_pose.pose.pose.position.x - 3.0) < 0.1) { // TODO: hard coded tolerance
                publishCmdVel(0.0, 0.0);
                state = States::DONE;
            }
            else {
                publishCmdVel(-0.3, 0.0);
            }
            break;

        case States::DONE:
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