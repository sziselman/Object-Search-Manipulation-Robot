#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
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
        ros::Publisher object_pub;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> x_locs;
        std::vector<double> y_locs;
        std::vector<double> z_locs;
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
            object_pub = n.advertise<scene_setup::BlockArray>("objects", 10, true);
        
            initialize_blocks();
        }

        /// \brief load_parameters 
        /// loads the parameters from the parameter server
        void load_parameters(void) {
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("frequency", frequency);
            n.getParam("x_locs", x_locs);
            n.getParam("y_locs", y_locs);
            n.getParam("z_locs", z_locs);

            return;
        }

        /// \brief initialize_blocks
        /// takes in the (x,y,z) locations and sets them as blocks
        void initialize_blocks(void) {

            std::cout << "INITIALIZING BLOCKS ++++++++++++++++++++++++++++++++++++\r" << std::endl;
            for (int i = 0; i < x_locs.size(); i++) {
                scene_setup::Block block;

                geometry_msgs::Pose block_pose;
                block_pose.position.x = x_locs[i];
                block_pose.position.y = y_locs[i];
                block_pose.position.z = z_locs[i];
                block_pose.orientation.w = 1.0;

                block.pose = block_pose;
                block.dimensions = object_dimensions;
                block.id = i;

                // std::cout << "block " << block.id << "\r" << std::endl;
                blocks.push_back(block);
            }

            scene_setup::BlockArray block_array;
            block_array.blocks = blocks;
            
            for (auto x : block_array.blocks) {
                std::cout << "block " << x.id << "\r" << std::endl;
            }
            object_pub.publish(block_array);

            return;
        }

        void main_loop(void) {
            using namespace greedy_search;

            ros::Rate loop_rate(frequency);
            // GreedySearch greedy = GreedySearch(blocks);

            while (ros::ok()) {

                std::cout << "loop\r" << std::endl;

                // calculate the utility of each block
                for (auto block : blocks) {
                    std::cout << block.id << "\r" << std::endl;

                    scene_setup::Visibility visibility_msg;
                    
                    visibility_msg.request.block = block;

                    manipulator_control::TrajectoryExecution traj_msg;
                    traj_msg.request.block = block;
                    
                    double visibility, exec_time;

                    visibility_client.call(visibility_msg);
                    execution_time_client.call(traj_msg);

                    // get the utility
                    if (visibility_client.call(visibility_msg) && execution_time_client.call(traj_msg)) {
                        visibility = visibility_msg.response.visibility;
                        std::cout << "visibility " << visibility << "\r" << std::endl;

                        if (traj_msg.response.success == true) {
                            exec_time = traj_msg.response.duration;
                            std::cout << "execution time " << exec_time << "\r" << std::endl;
                            block.utility = visibility/exec_time;
                        }
                        else {
                            block.utility = 1e5;
                        }
                    }
                    // block.utility = visibility / exec_time;
                    std::cout << "utility " << block.utility << "\r" << std::endl;
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
