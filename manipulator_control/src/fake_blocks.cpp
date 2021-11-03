#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>

#include "manipulator_control/TrajectoryExecution.h"

class FakeManipulatorBlocks {
    private:
        // ros objects
        ros::NodeHandle n;
        ros::ServiceClient trajectory_client;

        // parameters
        int frequency;
        std::vector<double> object_dimensions;

    public:
        FakeManipulatorBlocks() {
            load_parameters();

            trajectory_client = n.serviceClient<manipulator_control::TrajectoryExecution>("get_execution_time");
        }

        void load_parameters(void) {
            n.getParam("frequency", frequency);
            n.getParam("object_dimensions", object_dimensions);

            return;
        }

        void main_loop(void) {
            ros::Rate loop_rate(frequency);

            geometry_msgs::Pose pose;
            pose.position.x = -0.25;
            pose.position.y = 0.25;
            pose.position.z = object_dimensions[2]/2;

            manipulator_control::TrajectoryExecution traj_msg;
            traj_msg.request.pose = pose;
            traj_msg.request.dimensions = object_dimensions;

            while(ros::ok()) {

                trajectory_client.call(traj_msg);

                if (trajectory_client.call(traj_msg)) {
                    std::cout << "trajectory execution time is " << traj_msg.response.duration << "\r" << std::endl;
                }
                else {
                    std::cout << "trajectory exectuion time is unavailable :(\r" << std::endl;
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