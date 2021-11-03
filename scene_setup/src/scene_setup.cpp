#include <ros/ros.h>
#include <vector>
#include <iostream>

#include "scene_setup/scene_geometry_lib.hpp"
#include "scene_setup/Visibility.h"


class ObjectScene {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceServer visibility_service;

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