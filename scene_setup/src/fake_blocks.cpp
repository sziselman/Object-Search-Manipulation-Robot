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

    public:
        FakeBlocks() {
            load_parameters();

            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
        }

        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("object_dimensions", object_dimensions);

            return;
        }

        void main_loop(void) {
            ros::Rate loop_rate(frequency);

            geometry_msgs::Pose pose;
            pose.position.x = -1.0;
            pose.position.y = 2.5;
            pose.position.z = object_dimensions[2]/2;

            scene_setup::Block block;
            block.dimensions = object_dimensions;
            block.pose = pose;

            scene_setup::Visibility visibility_msg;
            visibility_msg.request.block = block;

            while (ros::ok()) {
                visibility_client.call(visibility_msg);

                if (visibility_client.call(visibility_msg)) {
                    std::cout << "visibility is " << visibility_msg.response.visibility << "\r" << std::endl;
                }
                else {
                    std::cout << "visibility unavailable :(\r" << std::endl;
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