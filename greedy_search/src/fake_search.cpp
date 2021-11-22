#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
#include "greedy_search/Utility.h"

#include "manipulator_control/TrajectoryExecution.h"

#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>

class FakeSearch {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceClient visibility_client;
        ros::ServiceClient execution_time_client;
        ros::ServiceClient utility_client;
        ros::ServiceServer utility_service;
        ros::Publisher object_pub;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;
        std::vector<double> object3_position;

        std::vector<std::vector<double>> object_positions;
        int frequency;

        // variables
        std::vector<scene_setup::Block> blocks;
        bool vis_received = false;
        bool traj_received = false;

    public:
        FakeSearch() {
            load_parameters();

            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
            execution_time_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
            utility_service = n.advertiseService("get_utility", &FakeSearch::utility, this);
            utility_client = n.serviceClient<greedy_search::Utility>("get_utility");
            object_pub = n.advertise<scene_setup::BlockArray>("objects", 10, true);
        
            load_blocks();
        }

        /// \brief load_parameters 
        /// loads the parameters from the parameter server
        void load_parameters(void) {
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("frequency", frequency);
            n.getParam("object1_position", object1_position);
            object_positions.push_back(object1_position);
            n.getParam("object2_position", object2_position);
            object_positions.push_back(object2_position);
            n.getParam("object3_position", object3_position);
            object_positions.push_back(object3_position);

            return;
        }

        /// \brief callback function for utility service
        /// \param req
        /// \param res
        /// \return boolean true
        bool utility(greedy_search::Utility::Request &req,
                     greedy_search::Utility::Response &res) {
            
            // create service messages
            scene_setup::Visibility vis_msg;
            manipulator_control::TrajectoryExecution traj_msg;

            // populate those messages
            vis_msg.request.block = req.block;
            traj_msg.request.block = req.block;

            visibility_client.call(vis_msg);
            execution_time_client.call(traj_msg);

            if (visibility_client.call(vis_msg) && execution_time_client.call(traj_msg)) {
                double visibility = vis_msg.response.visibility;
                std::cout << "visibility of block " << req.block.id << " is " << visibility << "\r" << std::endl;

                // if planned removal of object is successful, get the trajectory execution time
                if (traj_msg.response.success) {
                    double exec_time = traj_msg.response.duration;
                    std::cout << "execution time of block " << req.block.id << " is " << exec_time << "\r" << std::endl;
                    res.success = true;
                    res.utility = visibility / exec_time;
                }
                // if planned removal of object is unsucessful, set the utility very low
                else {
                    res.success = false;
                    res.utility = -1e4;
                }
            }

            return true;
        }

        /// \brief load_blocks
        /// takes in the (x,y,z) locations and sets them as blocks
        void load_blocks(void) {

            int id = 1;

            for (auto pos : object_positions) {
                scene_setup::Block block;
                block.pose.position.x = pos[0];
                block.pose.position.y = pos[1];
                block.pose.position.z = pos[2];
                block.pose.orientation.w = 1.0;
                block.dimensions = object_dimensions;
                block.id = id;
                id++;

                blocks.push_back(block);
            }

            scene_setup::BlockArray block_array;
            block_array.blocks = blocks;
            
            object_pub.publish(block_array);

            return;
        }

        /// \brief main_loop
        /// the main loop that spins 
        void main_loop(void) {
            using namespace greedy_search;

            ros::Rate loop_rate(frequency);

            while (ros::ok()) {
                std::cout << "loop\r" << std::endl;

                // calculate the utility of each block
                for (auto block : blocks) {
                    std::cout << block.id << "\r" << std::endl;

                    greedy_search::Utility util_msg;
                    util_msg.request.block = block;

                    utility_client.call(util_msg);

                    if (utility_client.call(util_msg)) {
                        if (util_msg.response.success) {
                            block.utility = util_msg.response.utility;
                            std::cout << "utility " << block.utility << "\r" << std::endl;
                        }
                        else {
                            block.utility = 0;
                            std::cout << "utility unsuccessful\r" << std::endl;
                        }
                    }
                }

                // // input the blocks into the greedy search object
                GreedySearch greedy(blocks);
                std::vector<scene_setup::Block> arrangement;
                arrangement = greedy.getArrangement();

                ros::spinOnce();
                loop_rate.sleep();
            }

            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fake_search");
    FakeSearch node;
    node.main_loop();
    return 0;
}
