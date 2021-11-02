#ifndef GREEDY_SEARCH_INCLUDE_GUARD_HPP
#define GREEDY_SEARCH_INCLUDE_GUARD_HPP

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>

namespace greedy_search
{
    /// \brief a 3D object with a position and orientation
    struct Block
    {
        std::vector<double> dimensions;
        geometry_msgs::Pose pose;
        double visibility;
        double execution_time;
        double utility;
        int id;

        /// \brief constructor that creates a block located at (0.0, 0.0)
        /// \return block at origin
        Block();

        /// \brief constructor that takes a position and orientation
        /// \param block_pose a six dimensional vector containing position and orientation
        /// \return a block at specified position and orientation
        Block(std::vector<double> & block_dimensions, geometry_msgs::Pose & block_pose, int block_id);

        void updateVisibility(void);

        void updateExecutionTime(void);
        /// \brief calculates the utility (V/T)
        /// \return the utility of the block
        double calculate_utility(void);

    };

    /// \brief class for Greedy Search
    /// the greedy algorithm ranks the accessible objects in a scene based on utility and removes the highest utility object
    class GreedySearch
    {
        private:
            std::vector<Block> objects;     // list of poses (position and orientation) for the visible objects within a scene
            double num;                     // number of objects visible
        public:
            /// \brief constructor for GreedySearch object
            /// \param blocks : a vector of poses that represent the blocks seen from the RealSense camera
            GreedySearch(std::vector<Block> & blocks);

            /// \brief updates the objects within the scene
            /// \param block_poses : a vector of poses that represent the blocks seen from the RealSense camera
            void update_objects(std::vector<Block> & blocks);

            /// \brief gets the optimal arrangement using greedy search
            /// \return a list of blocks in order of arrangement
            std::vector<Block> get_arrangement(void);
    };
}

#endif