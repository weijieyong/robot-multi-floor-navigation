#pragma once
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <multi_floor_navigation/MapSwitcher.h>

class MapSwitcher{
    private:
        ros::Publisher map_pub;
        ros::Subscriber map0_sub, map1_sub;
        ros::ServiceServer switch_map_server;

        nav_msgs::OccupancyGrid map0, map1, map;
        void map0Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        bool switchMapCallback(multi_floor_navigation::MapSwitcher::Request &req, 
                               multi_floor_navigation::MapSwitcher::Response &res);

    public:
        MapSwitcher();
        void initialize (ros::NodeHandle& n);

};