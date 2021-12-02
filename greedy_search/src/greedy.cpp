#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
#include "greedy_search/StartSearch.h"

#include "manipulator_control/TrajectoryExecution.h"
#include "manipulator_control/RemoveObject.h"

#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>

#include <std_srvs/Empty.h>

#include <vector>
#include <string>

class Greedy {
    private:
        // ros objects
        ros::NodeHandle n;

        ros::Publisher object_marker_pub;

        ros::ServiceClient visibility_client;
        ros::ServiceClient execution_time_client;
        ros::ServiceClient remove_object_client;

        ros::ServiceServer search_service;

        ros::Subscriber object_sub;

        // parameters
        int frequency;

        // variables
        std::vector<scene_setup::Block> blocks;

    public:
        Greedy() {
            load_parameters();

            object_marker_pub = n.advertise<visualization_msgs::MarkerArray>("object_markers", 10, true);

            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
            execution_time_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
            remove_object_client = n.serviceClient<manipulator_control::RemoveObject>("remove_object");
            
            search_service = n.advertiseService("start_search", &Greedy::start_search, this);

            object_sub = n.subscribe("objects", 10, &Greedy::object_callback, this);
        }

        /// \brief loads the parameters from the parameter server
        void load_parameters(void) {
            n.getParam("frequency", frequency);
            return;
        }
        
        /// \brief callback function for subscriber to the /objects topic to read in the objects that are seen
        void object_callback(const scene_setup::BlockArray msg) {
            blocks = msg.blocks;
            return;
        }

        /// \brief updates the utility of a block
        /// \param block : the block to calculate the utility of
        /// \return the block with updated utility
        scene_setup::Block update_utility(scene_setup::Block block) {
            scene_setup::Visibility vis_msg;
            manipulator_control::TrajectoryExecution traj_msg;

            vis_msg.request.block = block;
            traj_msg.request.block = block;

            if (visibility_client.call(vis_msg) && execution_time_client.call(traj_msg)) {
                double visibility = vis_msg.response.visibility;
                std::cout << "visibility of block " << block.id << " is " << visibility << "\r" << std::endl;

                // if planned removal of object is successful, get the trajectory execution time
                if (traj_msg.response.success) {
                    double exec_time = traj_msg.response.duration;
                    std::cout << "execution time of block " << block.id << " is " << exec_time << "\r" << std::endl;
                    block.utility = visibility / exec_time;

                    std::cout << "utility of block " << block.id << " is " << block.utility << "\r" << std::endl;
                }
                // if planned removal of object is unsucessful, set the utility very low
                else {
                    block.utility = 0;
                }
            }
            return block;
        }

        /// \brief updates the object markers in rviz, the object with the highest utility will be highlighted in yellow
        void update_object_markers(scene_setup::BlockArray array) {
            visualization_msgs::MarkerArray marker_arr;
            visualization_msgs::Marker obj;

            obj.header.frame_id = "base_link";
            obj.action = 3;
            marker_arr.markers.push_back(obj);

            object_marker_pub.publish(marker_arr);

            marker_arr.markers.clear();
            
            // loop through all blocks
            for (int i = 0; i < array.blocks.size(); i++) {

                scene_setup::Block block = array.blocks[i];

                obj.ns = "objects";
                obj.type = 1;
                obj.action = 0;
                obj.id = block.id;
                obj.pose = block.pose;
                obj.scale.x = block.dimensions[0];
                obj.scale.y = block.dimensions[1];
                obj.scale.z = block.dimensions[2];
                obj.color.a = 1.0;

                if (i == 0) {
                    obj.color.r = 253./255.;
                    obj.color.g = 253./255.;
                    obj.color.b = 150./255.;
                }
                else {
                    obj.color.r = 250./255.;
                    obj.color.g = 218./255.;
                    obj.color.b = 221./255.;
                }

                marker_arr.markers.push_back(obj);
            }

            object_marker_pub.publish(marker_arr);

            return;
        }

        /// \brief callback function for /start_service topic 
        /// gets the utility of each object and sorts into arrangment
        bool start_search(greedy_search::StartSearch::Request &req,
                          greedy_search::StartSearch::Response &res) {
            
            std::cout << "calculating utility of blocks\r" << std::endl;

            std::vector<scene_setup::Block> updated_blocks;
            
            // calculate the utility for each block
            for (auto block : blocks) {
                // calculate the utility of the block
                updated_blocks.push_back(update_utility(block));
            }

            blocks = updated_blocks;

            std::cout << "performing greedy search on blocks\r" << std::endl;

            greedy_search::GreedySearch greedy(blocks);
            std::vector<scene_setup::Block> arrangement;
            arrangement = greedy.get_arrangement();

            scene_setup::BlockArray arr;
            arr.blocks = arrangement;

            update_object_markers(arr);

            res.arrangement = arr;

            std::cout << "found arrangement\r" << std::endl;

            for (auto block : res.arrangement.blocks) {
                std::cout << "block " << block.id << " utility " << block.utility << "\r" << std::endl;
            }

            manipulator_control::RemoveObject rm_msg;
            rm_msg.request.block = res.arrangement.blocks[0];

            if (remove_object_client.call(rm_msg)) {
                std::cout << "removing object " << rm_msg.request.block.id << "\r" << std::endl;
            }

            return true;
        }

        /// \brief main_loop
        /// the main loop that spins 
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
    ros::init(argc, argv, "fake_search");
    Greedy node;
    node.main_loop();
    return 0;
}
