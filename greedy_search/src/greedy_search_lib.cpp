#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    GreedySearch::GreedySearch() {
        num = 0;
    }

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> blocks) {
        num = blocks.size();
        objects = blocks;
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