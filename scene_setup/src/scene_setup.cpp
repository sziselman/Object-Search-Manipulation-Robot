#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "scene_setup/scene_geometry_lib.hpp"

int main(int argc, char* argv[])
{
    using namespace scene;

    ros::init(argc, argv, "fake_scene_setup");
    ros::NodeHandle n;

    std::vector<double> dims_f{4.0, 3.0};
    double dist_f = 3.5;
    double dist_r = 7.5;

    // setup a scene based on the dimensions
    Scene scene = Scene(dims_f, dist_f, dist_r);

    std::vector<double> object_dims{1.0, 1.0, 1.0};
    geometry_msgs::Point object_pos;
    object_pos.x = 0.5;
    object_pos.y = 4.0;
    object_pos.z = object_dims[2] / 2;

    double visibility = scene.getObjectVisibility(object_dims, object_pos);

    std::cout << visibility << std::endl;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}