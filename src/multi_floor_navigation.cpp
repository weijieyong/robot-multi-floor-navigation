#include "robot-multi-floor-navigation/multi_floor_navigation.h"


MultiFloorNavigation::MultiFloorNavigation() :
    state(MultiFloorNavigation::States::INIT_POSE)
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
}

void MultiFloorNavigation::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    curr_pose.pose = msg->pose;
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

void MultiFloorNavigation::execute(){
    switch(state){
        case MultiFloorNavigation::States::INIT_POSE:
            initializeRobotPose(4.0, -5.0, 0.5, 0.0); // TODO: hard-coded
            state = MultiFloorNavigation::States::DONE;
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