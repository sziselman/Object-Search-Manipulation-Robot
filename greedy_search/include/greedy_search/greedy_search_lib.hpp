#ifndef GREEDY_SEARCH_INCLUDE_GUARD_HPP
#define GREEDY_SEARCH_INCLUDE_GUARD_HPP

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "scene_setup/Block.h"

#include <vector>
#include <string>

namespace greedy_search
{
    /// \brief class for Greedy Search
    /// the greedy algorithm ranks the accessible objects in a scene based on utility and removes the highest utility object
    class GreedySearch
    {
        private:
            std::vector<scene_setup::Block> objects;    // list of poses (position and orientation) for the visible objects within a scene
            double num;                                 // number of objects visible
        
        public:
            /// \brief constructor for GreedySearch object
            GreedySearch();

            /// \brief constructor for GreedySearch object
            /// \param blocks : a vector of poses that represent the blocks seen from the RealSense camera
            GreedySearch(std::vector<scene_setup::Block> blocks);

            /// \brief updates the objects within the scene
            /// \param block_poses : a vector of poses that represent the blocks seen from the RealSense camera
            void updateObjects(std::vector<scene_setup::Block> blocks);

            /// \brief gets the optimal arrangement using greedy search
            /// \return a list of blocks in order of arrangement
            std::vector<scene_setup::Block> getArrangement(void);
    };
}

#endif