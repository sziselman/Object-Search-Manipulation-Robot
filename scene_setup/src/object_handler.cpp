#include <ros/ros.h>
#include <map>
#include <vector>

#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

class ObjectHandler {
    private:
        // pubs, subs, servs, etc.
        ros::NodeHandle n;
        ros::Publisher object_pub;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;
        std::vector<double> object3_position;
        std::vector<std::vector<double>> object_positions;
        int frequency;

        // set that takes care of the objects within the scene
        // maps the block id to the block
        std::map<int, scene_setup::Block> object_map = {};

        // variables
        scene_setup::BlockArray block_arr;

    public:
        ObjectHandler() {
            load_parameters();
            initialize_dictionary();
            object_pub = n.advertise<scene_setup::BlockArray>("objects", 10, true);
        }

        void load_parameters(void) {
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("object1_position", object1_position);
            object_positions.push_back(object1_position);
            n.getParam("object2_position", object2_position);
            object_positions.push_back(object2_position);
            n.getParam("object3_position", object3_position);
            object_positions.push_back(object3_position);
            n.getParam("frequency", frequency);
        }

        void initialize_dictionary(void) {
            int id = 1;
            for (auto pos : object_positions) {
                // initialize block object
                scene_setup::Block block;
                block.pose.position.x = pos[0];
                block.pose.position.y = pos[1];
                block.pose.position.z = pos[2];
                block.pose.orientation.w = 1.0;
                block.dimensions = object_dimensions;
                block.id = id;
                id++;
                
                // insert block into the map
                object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));
            }
        }

        void main_loop(void) {
            ros::Rate loop_rate(100);

            while (ros::ok()) {

                for (auto const& val : object_map) {
                    block_arr.blocks.push_back(val.second);
                }

                object_pub.publish(block_arr);
                
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