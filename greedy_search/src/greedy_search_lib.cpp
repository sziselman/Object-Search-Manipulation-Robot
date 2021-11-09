#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    // Block::Block() {};


    // Block::Block(scene_setup::Block &block_) {
    //     block = block_;
    // }

    // bool Block::operator < (const Block &right) {
    //     return block.utility < right.block.utility;
    // }

    GreedySearch::GreedySearch() {
        num = 0;
    }

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> blocks) {
        num = blocks.size();
        // objects is list of blocks within scene
        objects = blocks;

        // loop through each object (block)
        // for (auto o : objects) {
        //     // create a Block object using that block
        //     Block block_object(o);
        //     // insert that Block object into the search's ordered_set
        //     arrangement.insert(block_object);
        // }
    }

    void GreedySearch::updateObjects(std::vector<scene_setup::Block> blocks) {
        num = blocks.size();
        objects = blocks;
    }

    std::vector<scene_setup::Block> GreedySearch::getArrangement(void) {
        std::vector<scene_setup::Block> arrangement;
        
        return arrangement;
    }

}