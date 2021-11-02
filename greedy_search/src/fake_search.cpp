#include <ros/ros.h>

#include "greedy_search/greedy_search_lib.hpp"
#include "manipulator_control/TrajectoryExecution.h"
#include "scene_setup/Visibility.h"

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

        // standard variables
        std::vector<double> dimensions{1.0, 1.0, 1.0};
        std::vector<visualization_msgs::Marker> objects;
        std::vector<greedy_search::Block> blocks;
        std::vector<greedy_search::Block> arrangement;

    public:
        FakeSearch() {
            initializePoses();
            markers2blocks();

            // arrangement_pub = n.advertise<std::vector<greedy_search::Block>>("arrangement", 10);
            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
            execution_time_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
        }

        void initializePoses(void) {
            // marker for block 1
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 0;

            geometry_msgs::Pose block_pose;
            block_pose.position.x = -1.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, 0.0);
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose = block_pose;

            objects.push_back(marker);

            // marker for block 2
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 1;

            block_pose.position.x = -0.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose = block_pose;

            objects.push_back(marker);

            // marker for block 3
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 2;

            block_pose.position.x = 0.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose = block_pose;

            objects.push_back(marker);

            // marker for block 4
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 3;

            block_pose.position.x = 1.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose = block_pose;

            objects.push_back(marker);

            return;
        }

        void markers2blocks(void) {
            for (auto object : objects) {
                greedy_search::Block block(dimensions, object.pose, object.id);
                blocks.push_back(block);
            }

            return;
        }

        void main_loop(void) {
            using namespace greedy_search;

            ros::Rate loop_rate(100);
            GreedySearch greedy = GreedySearch(blocks);

            while (ros::ok()) {
                
                // calculate the occluded visibility of the object

                for (auto block : blocks) {
                    scene_setup::Visibility vis_msg;
                    vis_msg.request.pose = block.pose;
                    vis_msg.request.dimensions = dimensions;
                    block.visibility = visibility_client.call(vis_msg);

                    manipulator_control::TrajectoryExecution exec_msg;
                    exec_msg.request.pose = block.pose;
                    block.execution_time = execution_time_client.call(exec_msg);

                    block.utility = block.visibility / block.execution_time;
                }
                
                ros::spinOnce();
                loop_rate.sleep();
            }

            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fake_search");
    // FakeSearch node;
    // node.main_loop();
    // return 0;
}
