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

    Block::Block(std::vector<double> block_dimensions, geometry_msgs::Pose block_pose, int block_id)
    {
        dimensions = block_dimensions;
        pose = block_pose;
        id = block_id;
    }

    double Block::calculate_utility(void)
    {
        // the number of target poses that become visible once the object is removed
        double visibility = 0;

        // estimate of time necessary to execute the trajectory of the robot
        double execution_time = 1;

        // the estimated utility of the block
        return visibility / execution_time;
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
        for (int i = 0; i < objects.size()-1; i++) {
            for (int j = i+1; j < objects.size(); j++) {

                // calculate the utility
                double utility1 = objects[i].calculate_utility();
                double utility2 = objects[j].calculate_utility();

                // if the utility of ind i is greater than utility of ind j, swap
                if (utility1 > utility2) {
                    Block temp = objects[i];
                    objects[i] = objects[j];
                    objects[j] = temp;
                }
            }
        }
        
        return objects;
    }

}