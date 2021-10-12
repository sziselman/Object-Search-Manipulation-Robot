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

    Block::Block(std::vector<double> & block_dimensions, geometry_msgs::Pose & block_pose)
    {
        dimensions = block_dimensions;
        pose = block_pose;
    }

    double Block::calculate_utility(void)
    {
        // the number of target poses that become visible once the object is removed
        double visible;

        // estimate of time necessary to execute the trajectory of the robot
        double time;

        // the estimated utility of the block
        return visible / time;
    }

    GreedySearch::GreedySearch(std::vector<Block> & blocks)
    {
        num = blocks.size();

        // // for each pose, initialize a block object
        // for (int i = 0; i < num; i++)
        // {
        //     Block block = Block(block_poses[i]);
        //     block.id = to_string(i);

        //     objects.push_back(block);
        // }
        objects = blocks;
    }

    void GreedySearch::update_objects(std::vector<Block> & blocks)
    {
        num = blocks.size();

        // for (int i = 0; i < num; i++)
        // {
        //     // objects[i] = Block::Block(block_poses[i]);
        //     Block::Block block = Block::Block(block_poses[i]);
        //     block.id = to_string(i);
        //     objects.push_back(block);
        // }
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