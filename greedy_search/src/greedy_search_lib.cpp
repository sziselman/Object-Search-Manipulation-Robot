#include "greedy_search/greedy_search_lib.hpp"

namespace greedy_search
{
    BlockStruct::BlockStruct() {}
    
    BlockStruct::BlockStruct(scene_setup::Block &b) : block(b), utility(b.utility) {}

    bool BlockStruct::operator < (const BlockStruct &right) {
        return utility > right.utility;
    }

    GreedySearch::GreedySearch() {}

    GreedySearch::GreedySearch(std::vector<scene_setup::Block> & blocks) : objects(blocks) {}

    std::vector<scene_setup::Block> GreedySearch::getArrangement(void) {

        // set that orders blocks based on utliity value
        std::vector<BlockStruct> ordered_blocks_vec;

        /****************************
         * Testing the operator
         * *************************/
        std::cout << "testing < operator \r" << std::endl;
        BlockStruct b1 = BlockStruct(objects[2]);
        BlockStruct b4 = BlockStruct(objects[3]);
        BlockStruct b5 = BlockStruct(objects[5]);
        std::cout << "b1 utility " << b1.utility << std::endl;
        std::cout << "b4 utility " << b4.utility << std::endl;
        std::cout << "b5 utility " << b5.utility << std::endl;

        std::cout << "b4 < b5 is " << (b4<b5) << std::endl;
        std::cout << "b5 < b4 is " << (b5<b4) << std::endl;
        std::cout << "b1 < b5 is " << (b1<b5) << std::endl;

        std::cout << "getting arrangement \r" << std::endl;

        /***************************
         * Testing adding each object as a BlockStruct
         * ************************/
        std::cout << "testing adding each object as a blockstruct \r" << std::endl;

        // loop through each object and insert it into the ordered set
        for (auto object : objects) {

            std::cout << "object " << object.id << std::endl;
            std::cout << "utility " << object.utility << std::endl;
            
            // create BlockStruct 
            BlockStruct b_struct(object);

            ordered_blocks_vec.push_back(b_struct);
        }

        std::cout << "testing sorting set/vector \r" << std::endl;

        std::sort(ordered_blocks_vec.begin(), ordered_blocks_vec.end());

        std::cout << "using vec +++++++++++++++++++++\r" << std::endl;
        for (int i = 0; i < ordered_blocks_vec.size(); i++) {
            // std::cout << "object id " << ordered_blocks_vec[i].block.utility << std::endl;
            arrangement.push_back(ordered_blocks_vec[i].block);
        }

        for (int i = 0; i < arrangement.size(); i++) {
            std::cout << "object id " << arrangement[i].id << std::endl;
        }
        return arrangement;
    }

}