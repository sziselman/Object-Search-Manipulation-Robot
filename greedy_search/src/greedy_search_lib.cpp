#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    GreedySearch::GreedySearch() {}

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> & blocks) : objects(blocks) {}

    std::vector<scene_setup::Block> GreedySearch::getArrangement(void) {
        std::vector<scene_setup::Block> copy;
        copy = objects;

        while (copy.size() > 0) {
            double max_utility = -1e5;
            scene_setup::Block max_block;
            // loop through each block and find the one with the largest utility
            for (auto block : copy) {
                if (block.utility > max_utility) {
                    max_block = block;
                }
            }

            // once the block with max utility is found, its added to the arrangement
            arrangement.push_back(max_block);
            // remove the block with the max utility
            remove(copy.begin(), copy.end(), max_block);
        }
        
        return arrangement;
    }

}