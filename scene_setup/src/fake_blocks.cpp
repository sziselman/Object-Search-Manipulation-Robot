#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>

#include "scene_setup/Visibility.h"
#include "scene_setup/Block.h"

class FakeBlocks {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceClient visibility_client;

        // parameters
        int frequency;
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;

        scene_setup::Block block1;
        scene_setup::Block block2;

    public:
        FakeBlocks() {
            load_parameters();
            load_blocks();

            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
        }

        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("object1_position", object1_position);
            n.getParam("object2_position", object2_position);

            return;
        }

        void load_blocks(void) {
            // block1 attributes
            block1.pose.position.x = object1_position[0];
            block1.pose.position.y = object1_position[1];
            block1.pose.position.z = object1_position[2];
            block1.dimensions = object_dimensions;
            block1.id = 1;

            // block2 attributes
            block2.pose.position.x = object2_position[0];
            block2.pose.position.y = object2_position[1];
            block2.pose.position.z = object2_position[2];
            block2.dimensions = object_dimensions;
            block2.id = 2;

            return;
        }
        void main_loop(void) {
            ros::Rate loop_rate(frequency);

            scene_setup::Visibility vis_msg_1;
            scene_setup::Visibility vis_msg_2;

            vis_msg_1.request.block = block1;
            vis_msg_2.request.block = block2;

            while (ros::ok()) {
                visibility_client.call(vis_msg_1);

                if (visibility_client.call(vis_msg_1)) {
                    std::cout << "visibility of object 1 is " << vis_msg_1.response.visibility << "\r" << std::endl;
                }
                else {
                    std::cout << "visibility of object 1 is unavailable :(\r" << std::endl;
                }

                visibility_client.call(vis_msg_2);

                if (visibility_client.call(vis_msg_2)) {
                    std::cout << "visibility of object 2 is " << vis_msg_2.response.visibility << "\r" << std::endl;
                }
                else {
                    std::cout << "visibility of object 2 is unavailable >:(\r" << std::endl;
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fake_blocks");
    FakeBlocks node;
    node.main_loop();
    return 0;
}