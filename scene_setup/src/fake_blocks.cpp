#include <ros/ros.h>
#include <scene_setup/Visibility.h>
#include <vector>
#include <geometry_msgs/Pose.h>

class FakeBlocks {
    private:
        ros::ServiceClient visibility_client;
    public:
        FakeBlocks() {
            ros::NodeHandle n;
            visibility_client = n.serviceClient<scene_setup::Visibility>("get_visibility");
        }

        void main_loop(void) {
            ros::Rate loop_rate(10);

            geometry_msgs::Pose pose;
            pose.position.x = -0.5;
            pose.position.y = 2.5;
            pose.position.z = 0.5;

            std::vector<double> dims{1.0, 1.0, 1.0};

            scene_setup::Visibility visibility_msg;
            visibility_msg.request.pose = pose;
            visibility_msg.request.dimensions = dims;

            while (ros::ok()) {
                visibility_client.call(visibility_msg);

                if (visibility_client.call(visibility_msg)) {
                    std::cout << "visibility is \r" << std::endl;
                }
                else {
                    std::cout << "visibility unavailable :(" << std::endl;
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