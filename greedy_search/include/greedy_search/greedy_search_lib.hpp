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

        /// \brief constructor for BlockStruct object
        BlockStruct();

        /// \brief constructor for BlockStruct object
        /// \param b : a scene_setup/Block msg
        BlockStruct(scene_setup::Block &b);

        /// \brief operator overload on < (less than)
        /// returns true if the lhs utility is less than the rhs utility
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