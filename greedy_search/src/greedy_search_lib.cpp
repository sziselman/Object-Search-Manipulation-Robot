#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    BlockStruct::BlockStruct() {}
    
    BlockStruct::BlockStruct(scene_setup::Block &b) : block(b), utility(b.utility) {}

    bool BlockStruct::operator < (const BlockStruct &right) {
        return utility < right.utility;
    }

    GreedySearch::GreedySearch() {}

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> & blocks) : objects(blocks) {
        std::cout << "initializing greedy search object\r" << std::endl;
    }

    std::vector<scene_setup::Block> GreedySearch::getArrangement(void) {
        std::set<BlockStruct*> ordered_blocks;

        std::cout << "getting arrangement \r" << std::endl;

        for (auto object : objects) {
            std::cout << "object " << object.id << std::endl;
            // create BlockStruct 
            BlockStruct* b_struct = new BlockStruct(object);

            // insert the BlockStruct to the ordered set
            ordered_blocks.insert(b_struct);
        }

        std::cout << "arrangement found\r" << std::endl;

        // loop through the ordered set and blocks in vector
        for (auto it : ordered_blocks) {
            std::cout << "object id " << it->block.id << std::endl;
            std::cout << "object utility " << it->block.utility << std::endl;
            arrangement.push_back(it->block);
        }

        return arrangement;
    }

}