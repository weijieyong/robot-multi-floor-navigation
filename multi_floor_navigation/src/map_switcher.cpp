#include "multi_floor_navigation/map_switcher.h"

MapSwitcher::MapSwitcher()
{
}

void MapSwitcher::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    map0_sub = n.subscribe("map0", 1, &MapSwitcher::map0Callback, this);
    map1_sub = n.subscribe("map1", 1, &MapSwitcher::map1Callback, this);

    switch_map_server = n.advertiseService("switch_map", &MapSwitcher::switchMapCallback, this);
}

void MapSwitcher::map0Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map0 = *msg;
}

void MapSwitcher::map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map1 = *msg;
}

bool MapSwitcher::switchMapCallback(multi_floor_navigation::MapSwitcher::Request &req, 
                                    multi_floor_navigation::MapSwitcher::Response &res){
    if(req.floor_no == 0) {
        map.header = map0.header;
        map.info = map0.info;
        map.data = map0.data;
    }
    else {
        map.header = map1.header;
        map.info = map1.info;
        map.data = map1.data;
    }
    map_pub.publish(map);

    res.success = true;
    res.message = "Successfully loaded map for floor: " + std::to_string(req.floor_no);

    return true;  
}

int main(int argc, char** argv) {   
    ros::init(argc, argv, "map_switcher_node");
    ros::NodeHandle n;

    MapSwitcher switch_map;
    switch_map.initialize(n);

    ros::Rate rate(5.0);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

