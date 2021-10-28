#include <ros/ros.h>
#include <greedy_search/greedy_search_lib.hpp>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

class FakeSensor {
    private:﻿
        ros::NodeHandle n;
        ros::Publisher arrangement_pub;

        // standard variables
        std::vector<double> dimensions{1.0, 1.0, 1.0};
        std::vector<visualization_msgs::Marker> objects;
        std::vector<greedy_search::Block> blocks;
        std::vector<greedy_search::Block> arrangement;

    public:
        FakeSensor() {
            initializePoses();
            markers2blocks();

            arrangement_pub = n.advertise<std::vector<Block>>("/arrangement", 10);
        }

        void initializePoses(void) {
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

            marker.pose.pose = block_pose;

            objects.push_back(marker);

            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 1;

            geometry_msgs::Pose block_pose;
            block_pose.position.x = -0.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, 0.0);
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose.pose = block_pose;

            objects.push_back(marker);

            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 2;

            geometry_msgs::Pose block_pose;
            block_pose.position.x = 0.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, 0.0);
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose.pose = block_pose;

            objects.push_back(marker);

            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = 3;

            geometry_msgs::Pose block_pose;
            block_pose.position.x = 1.5;
            block_pose.position.y = 2.5;
            block_pose.position.z = dimensions[2]/2;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, 0.0);
            block_pose.orientation = tf2::toMsg(quat);

            marker.pose.pose = block_pose;

            objects.push_back(marker);

            return;
        }

        void markers2blocks(void) {
            for (auto object : objects) {
                Block block(dimensions, object.pose.pose, object.id);
                blocks.push_back(block);
            }

            return;
        }

        void main_loop(void) {
            using namespace greedy_search;

            ros::Rate loop_rate(10);
            GreedySearch greedy = GreedySearch(blocks);

            while (ros::ok()) {
                
                // calculate the occluded visibility of the object
                
                ros::spinOnce();
                loop_rate.sleep();
            }

            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fake_sensor");
    FakeSensor node;
    node.main_loop();
    return 0;
}
