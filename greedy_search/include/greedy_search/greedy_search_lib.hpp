#ifndef GREEDY_SEARCH_INCLUDE_GUARD_HPP
#define GREEDY_SEARCH_INCLUDE_GUARD_HPP

#include <geometry_msgs/Pose.h>
#include "scene_setup/Block.h"

#include <vector>
#include <string>
#include <set>

namespace greedy_search
{
    struct BlockStruct {
        scene_setup::Block block;
        double utility;

        BlockStruct();

        BlockStruct(scene_setup::Block &b);

        bool operator < (const BlockStruct &right);
        
    };

    /// \brief class for Greedy Search
    /// the greedy algorithm ranks the accessible objects in a scene based on utility and removes the highest utility object
    class GreedySearch
    {
        private:
            std::vector<scene_setup::Block> objects;        // list of blocks
            std::vector<scene_setup::Block> arrangement;
        
        public:
            /// \brief constructor for GreedySearch object
            GreedySearch();

            /// \brief constructor for GreedySearch object
            /// \param blocks : a vector of poses that represent the blocks seen from the RealSense camera
            GreedySearch(std::vector<scene_setup::Block> & blocks);

            /// \brief gets the optimal arrangement using greedy search
            /// \return a list of blocks in order of arrangement
            std::vector<scene_setup::Block> getArrangement(void);
    };
}

#endif