#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>

#include "manipulator_control/TrajectoryExecution.h"
#include "scene_setup/Block.h"

class FakeManipulatorBlocks {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceClient trajectory_client;

        // parameters
        int frequency;
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;

        scene_setup::Block block1, block2;

    public:
        FakeManipulatorBlocks() {
            load_parameters();
            load_blocks();

            trajectory_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
        }

        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("object_dimensions", object_dimensions);
            n.getParam("object1_position", object1_position);
            n.getParam("object2_position", object2_position);

            return;
        }

        void load_blocks(void) {
            block1.pose.position.x = object1_position[0];
            block1.pose.position.y = object1_position[1];
            block1.pose.position.z = object1_position[2];
            block1.pose.orientation.w = 1;
            block1.dimensions = object_dimensions;
            block1.id = 1;

            block2.pose.position.x = object2_position[0];
            block2.pose.position.y = object2_position[1];
            block2.pose.position.z = object2_position[2];
            block2.pose.orientation.w = 1;
            block2.dimensions = object_dimensions;
            block2.id = 2;

            return;
        }

        void main_loop(void) {
            ros::Rate loop_rate(frequency);

            manipulator_control::TrajectoryExecution traj_msg;
            traj_msg.request.block = block1;

            while(ros::ok()) {

                trajectory_client.call(traj_msg);

                if (trajectory_client.call(traj_msg)) {

                    if (traj_msg.response.success == true) {
                        std::cout << "successful trajectory execution time is " << traj_msg.response.duration << "\r" << std::endl;
                    }
                    else {
                        std::cout << "unsuccessful trajectory execution\r" << std::endl;
                    }
                }

                ros::spinOnce();
                loop_rate.sleep();
            }

            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fake_manipulator_blocks");
    FakeManipulatorBlocks node;
    node.main_loop();
    return 0;
}