#include <ros/ros.h>
#include <greedy_search/greedy_search_lib.hpp>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

int main(int argc, char* argv[])
{
    using namespace greedy_search;

    ros::init(argc, argv, "fake_real_sense");
    ros::NodeHandle n;

    std::vector<double> dimensions;
    std::vector<std::vector<double>> poses;
    std::vector<Block> blocks;
    std::vector<Block> arrangement;

    n.getParam("dimensions", dimensions);
    n.getParam("block_poses", poses);

    ros::Publisher object_pub = n.advertise<std::vector<std::vector<double>>>("/objects", 10);

    ros::Rate loop_rate(100);

    // create a vector of poses from the pose values loaded from the yaml file
    for (auto pose: poses)
    {
        geometry_msgs::Pose block_pose;
        block_pose.position.x = pose[0];
        block_pose.position.y = pose[1];
        block_pose.position.z = pose[2];

        tf2::Quaternion quat;
        quat.setRPY(pose[3], pose[4], pose[5]);
        block_pose.orientation = tf2::toMsg(quat);

        Block block = Block(dimensions, block_pose);
        blocks.push_back(block);
    }

    GreedySearch greedy = GreedySearch(blocks);

    while(ros::ok())
    {
        ros::spinOnce();

        arrangement = greedy.get_arrangement();
        ROS_INFO("arrangement found");
        for (auto block : arrangement)
        {
            ROS_INFO("block order is: ");
            ROS_INFO("block %i", block.id);
        }
    }
}