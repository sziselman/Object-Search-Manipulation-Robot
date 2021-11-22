#include <ros/ros.h>
#include <map>
#include <vector>

#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

#include "scene_setup/RemoveObjectId.h"

class ObjectHandler {
    private:
        // pubs, subs, servs, etc.
        ros::NodeHandle n;
        ros::Publisher object_pub;
        ros::ServiceServer remove_object_id_service;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;
        std::vector<double> object3_position;
        std::vector<double> goal_object_position;
        std::vector<std::vector<double>> object_positions;
        int frequency;

        // set that takes care of the objects within the scene
        // maps the block id to the block
        std::map<int, scene_setup::Block> object_map = {};

        scene_setup::BlockArray block_arr;

    public:
        ObjectHandler() {
            load_parameters();
            object_pub = n.advertise<scene_setup::BlockArray>("objects", 10, true);
            remove_object_id_service = n.advertiseService("remove_object_id", &ObjectHandler::remove_object_id, this);
            initialize_dictionary();
        }

        /// \brief load_parameters : loading the parameters from server
        void load_parameters(void) {
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("object1_position", object1_position);
            object_positions.push_back(object1_position);
            n.getParam("object2_position", object2_position);
            object_positions.push_back(object2_position);
            n.getParam("object3_position", object3_position);
            object_positions.push_back(object3_position);
            n.getParam("frequency", frequency);

            n.getParam("goal_object_position", goal_object_position);
        }

        /// \brief initialize_dictionary
        /// reads block locations from parameter server, inserts them into a dictionary
        void initialize_dictionary(void) {
            int id = 1;
            // add the objects within the scene
            for (auto pos : object_positions) {
                // initialize block object
                scene_setup::Block block;
                block.pose.position.x = pos[0];
                block.pose.position.y = pos[1];
                block.pose.position.z = pos[2];
                block.pose.orientation.w = 1.0;
                block.dimensions = object_dimensions;
                block.id = id;
                goal = false;
                id++;
                
                // insert block into the map
                if (object_map.find(block.id) == object_map.end()) {
                    object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));
                }
            }
        }

        /// \brief remove_object_id
        /// removes the object with request.id from the dictionary
        bool remove_object_id(scene_setup::RemoveObjectId::Request &req,
                              scene_setup::RemoveObjectId::Response &res) {
            
            std::cout << "erasing object of id " << req.id << "\r" << std::endl;
            object_map.erase(req.id);
            
            return true;
        }

        /// \brief main_loop
        /// the main loop that gets executed after each spin
        void main_loop(void) {
            ros::Rate loop_rate(100);

            while (ros::ok()) {

                for (auto const& val : object_map) {
                    block_arr.blocks.push_back(val.second);
                }

                // scene_setup::Block goal_block;
                // goal_block.pose.position.x = goal_object_position[0];
                // goal_block.pose.position.y = goal_object_position[1];
                // goal_block.pose.position.z = goal_object_position[2];
                // goal_block.orientation.w = 1.0;
                // goal_block.dimensions = object_dimensions;
                // block.id = 0
                // goal = true;

                // block_arr.blocks.push_back(goal_block);
                
                object_pub.publish(block_arr);

                block_arr.blocks.clear();
                
                ros::spinOnce();
                loop_rate.sleep();
            }
            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "object_handler");
    ObjectHandler node;
    node.main_loop();
    return 0;
}