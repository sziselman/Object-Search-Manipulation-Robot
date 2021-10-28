#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    Block::Block()
    {
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = dimensions[2]/2;
        
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        pose.orientation = tf2::toMsg(quat);
    }

    Block::Block(std::vector<double> & block_dimensions, geometry_msgs::Pose & block_pose, int block_id)
    {
        dimensions = block_dimensions;
        pose = block_pose;
        id = block_id;
    }

    double Block::calculate_utility(void)
    {
        // the number of target poses that become visible once the object is removed
        double visibility;

        // estimate of time necessary to execute the trajectory of the robot
        double execution_time;

        // the estimated utility of the block
        return visible / time;
    }

    GreedySearch::GreedySearch(std::vector<Block> & blocks)
    {
        num = blocks.size();
        objects = blocks;
    }

    void GreedySearch::update_objects(std::vector<Block> & blocks)
    {
        num = blocks.size();
        objects = blocks;
    }

    std::vector<Block> GreedySearch::get_arrangement(void)
    {
        std::vector<Block> arrangement;

        // calculate the utility of each object
        for (auto object : objects)
        {
            object.calculate_utility();
        }
        return arrangement;
    }

}