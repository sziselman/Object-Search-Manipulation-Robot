#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "scene_setup/scene_geometry_lib.hpp"
#include "scene_setup/Visibility.h"


class ObjectScene {
    private:
        ros::NodeHandle n;
        ros::ServiceServer visibility_service;

        std::vector<double> front_dims{4.0, 3.0};
        double dist_front = 2.0;
        double dist_rear = 4.5;
        std::vector<double> object_dims{1.0, 1.0, 1.0};
        
        scene::Scene search_scene;

    public:
        ObjectScene() {
            visibility_service = n.advertiseService("get_visibility", &ObjectScene::visibility, this);
            search_scene = scene::Scene(front_dims, dist_front, dist_rear);
        }

        bool visibility(scene_setup::Visibility::Request& req,
                        scene_setup::Visibility::Response& res) {

            res.visibility = search_scene.getObjectVisibility(object_dims, req.pose);
            return true;
        }

        void main_loop(void) {

            ros::Rate loop_rate(10);

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