#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
#include "greedy_search/Utility.h"
#include "greedy_search/StartSearch.h"

#include "manipulator_control/TrajectoryExecution.h"

#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>

#include <std_srvs/Empty.h>

#include <vector>
#include <string>

class Greedy {
    private:
        // ros objects
        ros::NodeHandle n;

        ros::ServiceClient visibility_client;
        ros::ServiceClient execution_time_client;
        ros::ServiceClient utility_client;

        ros::ServiceServer utility_service;
        ros::ServiceServer search_service;

        ros::Subscriber object_sub;

        // parameters
        int frequency;

        // variables
        std::vector<scene_setup::Block> blocks;
        bool vis_received = false;
        bool traj_received = false;

    public:
        Greedy() {
            load_parameters();

            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
            execution_time_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
            utility_client = n.serviceClient<greedy_search::Utility>("get_utility");

            utility_service = n.advertiseService("get_utility", &Greedy::utility, this);
            search_service = n.advertiseService("start_search", &Greedy::start_search, this);

            object_sub = n.subscribe("objects", 10, &Greedy::object_callback, this);
        }

        /// \brief load_parameters 
        /// loads the parameters from the parameter server
        void load_parameters(void) {
            n.getParam("frequency", frequency);
            return;
        }
        
        /// \brief object_callback
        /// subscribes to the /objects topic to read in the objects that are seen
        void object_callback(const scene_setup::BlockArray msg) {
            std::vector<scene_setup::Block> incoming_blocks;

            for (auto block : msg.blocks) {
                incoming_blocks.push_back(block);
            }

            blocks = incoming_blocks;

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

        /// \brief callback function for /start_service topic 
        /// gets the utility of each object and sorts into arrangment
        bool start_search(greedy_search::StartSearch::Request &req,
                          greedy_search::StartSearch::Response &res) {

            using namespace greedy_search;
            
            std::cout << "calculating utility of blocks\r" << std::endl;

            for (auto block : blocks) {
                greedy_search::Utility util_msg;
                util_msg.request.block = block;

                utility_client.call(util_msg);

                if (utility_client.call(util_msg)) {
                    if (util_msg.response.success) {
                        block.utility = util_msg.response.utility;
                        std::cout << "utility of block " << block.id << " is " << block.utility << "\r" << std::endl;
                    }
                    else {
                        block.utility = 0;
                        std::cout << "utility unsuccessful\r" << std::endl;
                    }
                }
            }

            std::cout << "performing greedy search on blocks\r" << std::endl;

            GreedySearch greedy(blocks);
            std::vector<scene_setup::Block> arrangement;
            arrangement = greedy.getArrangement();

            scene_setup::BlockArray arr;
            arr.blocks = arrangement;

            res.arrangement = arr;

            for (auto block : res.arrangement.blocks) {
                std::cout << "block " << block.id << " utility " << block.utility << "\r" << std::endl;
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
