#include <ros/ros.h>

#include <vector>
#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"
#include "greedy_search/greedy_search_lib.hpp"

class TestGreedy {
    private:
        ros::NodeHandle n;
    public:
        TestGreedy() {

        }

        void main_loop(void) {
            using namespace greedy_search;

            scene_setup::Block block1, block2, block3, block4, block5, block6;
            block1.id = 1;
            block1.utility = 10;

            block2.id = 2;
            block2.utility = 20;

            block3.id = 3;
            block3.utility = 30;

            block4.id = 4;
            block4.utility = 40;


            block5.id = 5;
            block5.utility = 400;

            block6.id = 6;
            block6.utility = -40;


            std::vector<scene_setup::Block> blocks{block3, block2, block1, block4, block6, block5};
            GreedySearch greedy(blocks);

            std::vector<scene_setup::Block> arrangement;
            arrangement = greedy.getArrangement();

            std::cout << "arangmenet : \r" << std::endl;
            
            for (auto a : arrangement) {
                std::cout << "object " << a.id << std::endl;
            }

            ros::spinOnce();
            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test_greedy");
    TestGreedy node;
    node.main_loop();
    return 0;
}