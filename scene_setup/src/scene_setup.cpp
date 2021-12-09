#include <ros/ros.h>
#include <vector>
#include <iostream>

#include "scene_setup/scene_geometry_lib.hpp"
#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"

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
        std::vector<double> focal_point;

        double front_plane_distance;
        double rear_plane_distance;

        int frequency;
        
        // variables
        scene::Scene search_scene;

    public:
        /// \brief constructor for ObjectScene object
        ObjectScene() {
            load_parameters();

            search_scene = scene::Scene(front_plane_dimensions, front_plane_distance, rear_plane_distance);

            visibility_service = n.advertiseService("get_visibility", &ObjectScene::visibility, this);
            
            marker_pub = n.advertise<visualization_msgs::MarkerArray>("scene_markers", 10, true);
        }

        /// \brief function that loads parameters from the parameter server
        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("front_plane_dimensions", front_plane_dimensions);
            n.getParam("front_plane_distance", front_plane_distance);
            n.getParam("rear_plane_distance", rear_plane_distance);
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("focal_point", focal_point);
            return;
        }

        /// \brief create markers in rviz that represent the geometry of the scene
        void load_scene_markers(void) {
            visualization_msgs::MarkerArray array;

            // front plane
            visualization_msgs::Marker marker;

            std::vector<geometry_msgs::Point> front_plane;
            geometry_msgs::Point point;
            point.x = -front_plane_dimensions[0]/2;
            point.y = front_plane_distance;
            point.z = focal_point[2];

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
            marker.id = 1;
            marker.ns = "scene";
            marker.type = 4;
            marker.action = visualization_msgs::Marker::ADD;
            marker.points = front_plane;
            marker.scale.x = 0.005;
            marker.scale.y = 0.005;
            marker.color.a = 1.0;
            marker.color.r = 250. / 255.;
            marker.color.g = 192. / 255.;
            marker.color.b = 221. / 255.;
            marker.frame_locked = true;

            array.markers.push_back(marker);

            // rear plane
            std::vector<double> rear_plane_dims = search_scene.get_rear_plane_dimensions();
            std::vector<geometry_msgs::Point> rear_plane;
            point.x = -rear_plane_dims[0]/2;
            point.y = rear_plane_distance;
            point.z = focal_point[2];

            rear_plane.push_back(point);

            point.x += rear_plane_dims[0];
            rear_plane.push_back(point);

            point.z += rear_plane_dims[1];
            rear_plane.push_back(point);

            point.x -= rear_plane_dims[0];
            rear_plane.push_back(point);

            point.z -= rear_plane_dims[1];
            rear_plane.push_back(point);

            marker.points = rear_plane;
            marker.id ++;
            array.markers.push_back(marker);

            // rays outward
            std::vector<geometry_msgs::Point> edge;
            edge.resize(2);

            for (int i = 0; i < front_plane.size(); i++) {
                edge[0] = front_plane[i];
                edge[1] = rear_plane[i];
                marker.id++;
                marker.points = edge;
                array.markers.push_back(marker);
            }

            marker_pub.publish(array);
            return;
        }

        /// \brief function for /get_visibility service, calculates the volume of occluded volume due to an object
        /// \param req - (scene_setup/Visibility/Request) contains a block that is occluding volume in the scene
        /// \param res - (scene_setup/Visibility/Response) contains a float that represents the volume of occluded space due to the object
        /// \returns the boolean value true (if the service is successful) or false (if the service fails)
        bool visibility(scene_setup::Visibility::Request& req,
                        scene_setup::Visibility::Response& res) {

            res.visibility = search_scene.get_object_visibility(req.block);
            return true;
        }

        /// \brief the main loop to be executed
        void main_loop(void) {

            ros::Rate loop_rate(frequency);
            load_scene_markers();

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