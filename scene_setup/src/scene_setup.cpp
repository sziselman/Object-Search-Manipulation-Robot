#include <ros/ros.h>
#include <vector>
#include <iostream>

#include "scene_setup/scene_geometry_lib.hpp"
#include "scene_setup/Visibility.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <vector>


class ObjectScene {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceServer visibility_service;
        ros::Publisher marker_pub;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> front_plane_dimensions;
        double front_plane_distance;
        double rear_plane_distance;

        int frequency;
        
        // variables
        scene::Scene search_scene;

    public:
        ObjectScene() {
            load_parameters();

            search_scene = scene::Scene(front_plane_dimensions, front_plane_distance, rear_plane_distance);

            visibility_service = n.advertiseService("get_visibility", &ObjectScene::visibility, this);
            marker_pub = n.advertise<visualization_msgs::MarkerArray>("scene_markers", 10, true);

            load_scene_markers();
        }

        /// \brief load_parameters
        /// function that loads parameters from the parameter server
        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("front_plane_dimensions", front_plane_dimensions);
            n.getParam("front_plane_distance", front_plane_distance);
            n.getParam("rear_plane_distance", rear_plane_distance);
            n.getParam("object_dimensions", object_dimensions);
            return;
        }

        void load_scene_markers(void) {
            visualization_msgs::MarkerArray array;

            // front plane
            visualization_msgs::Marker marker;

            std::vector<geometry_msgs::Point> front_plane;
            geometry_msgs::Point point;
            point.x = -front_plane_dimensions[0]/2;
            point.y = front_plane_distance;
            point.z = 0.0;

            front_plane.push_back(point);

            point.x += front_plane_dimensions[0];
            front_plane.push_back(point);

            point.z += front_plane_dimensions[1];
            front_plane.push_back(point);

            point.x -= front_plane_dimensions[0];
            front_plane.push_back(point);

            point.z -= front_plane_dimensions[1];
            front_plane.push_back(point);

            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "scene";
            marker.type = 4;
            marker.action = visualization_msgs::Marker::ADD;
            marker.points = front_plane;
            marker.scale.x = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 250. / 255.;
            marker.color.g = 192. / 255.;
            marker.color.b = 221. / 255.;
            marker.frame_locked = true;

            array.markers.push_back(marker);

            // // rear plane
            // std::vector<double> rear_plane_dims = search_scene.getRearPlaneDimensions();
            // std::vector<geometry_msgs::Point> rear_plane;
            // point.x = -rear_plane_dims[0]/2;
            // point.y = rear_plane_distance;
            // point.z = 0.0;

            // rear_plane.push_back(point);

            // point.x += rear_plane_dims[0];
            // rear_plane.push_back(point);

            // point.z += rear_plane_dims[1];
            // rear_plane.push_back(point);

            // point.x -= rear_plane_dims[0];
            // rear_plane.push_back(point);

            // point.z -= rear_plane_dims[1];
            // rear_plane.push_back(point);

            // marker.points = rear_plane;
            // array.markers.push_back(marker);

            marker_pub.publish(array);
            return;
        }

        bool visibility(scene_setup::Visibility::Request& req,
                        scene_setup::Visibility::Response& res) {

            res.visibility = search_scene.getObjectVisibility(object_dimensions, req.pose);
            return true;
        }

        void main_loop(void) {

            ros::Rate loop_rate(frequency);

            while (ros::ok()) {
                
                ros::spinOnce();
                loop_rate.sleep();
            }
            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "scene_setup");
    ObjectScene node;
    node.main_loop();
    return 0;
}