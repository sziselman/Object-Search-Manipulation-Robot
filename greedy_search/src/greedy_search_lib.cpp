#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    BlockStruct::BlockStruct() {}
    
    BlockStruct::BlockStruct(scene_setup::Block &b) : block(b), utility(b.utility) {}

    bool BlockStruct::operator < (const BlockStruct &right) {
        return utility < right.utility;
    }

    bool BlockStruct::operator > (const BlockStruct &right) {
        return utility > right.utility;
    }

    GreedySearch::GreedySearch() {}

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> & blocks) : objects(blocks) {}

    std::vector<scene_setup::Block> GreedySearch::getArrangement(void) {

        // set that orders blocks based on utliity value
        std::vector<BlockStruct> ordered_blocks_vec;

        // loop through each object and insert it into the ordered set
        for (auto object : objects) {
            BlockStruct b_struct(object);
            ordered_blocks_vec.push_back(b_struct);
        }

        // sort the vector of blockstructs
        std::sort(ordered_blocks_vec.begin(), ordered_blocks_vec.end());


        for (auto block_struct : ordered_blocks_vec) {
            arrangement.push_back(block_struct.block);
        }

        std::reverse(arrangement.begin(), arrangement.end());

        return arrangement;
    }

}