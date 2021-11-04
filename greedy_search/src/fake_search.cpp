#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
#include "manipulator_control/TrajectoryExecution.h"
#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>

class FakeSearch {
    private:
        ros::NodeHandle n;
        // ros::Publisher arrangement_pub;
        ros::ServiceClient visibility_client;
        ros::ServiceClient execution_time_client;

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
            initialize_blocks();

            // // arrangement_pub = n.advertise<std::vector<greedy_search::Block>>("arrangement", 10);
            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
            execution_time_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
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

                blocks.push_back(block);
            }

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
                    visibility_msg.request.pose = block.pose;
                    visibility_msg.request.dimensions = object_dimensions;

                    manipulator_control::TrajectoryExecution traj_msg;
                    traj_msg.request.pose = block.pose;
                    traj_msg.request.dimensions = object_dimensions;

                    double visibility, exec_time;

                    visibility_client.call(visibility_msg);

                    // while (visibility_client.call(visibility_msg) != true) {
                    //     ;
                    // }

                    execution_time_client.call(traj_msg);

                    // while (execution_time_client.call(traj_msg) != true) {
                    //     ;
                    // }

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
