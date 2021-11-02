#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

#include "manipulator_control/TrajectoryExecution.h"


class ManipulatorArm {
    private:
        // ros variables
        ros::NodeHandle n;
        ros::Publisher pincer_pub;
        ros::ServiceServer execution_time_service;

        // move it variables
        moveit::planning_interface::MoveGroupInterface arm_move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const moveit::core::JointModelGroup* arm_model_group;

        moveit_visual_tools::MoveItVisualTools visual_tools;

    public:
        ManipulatorArm(std::string planning_group, std::string frame) : arm_move_group(planning_group),
                                                                        visual_tools(frame) {

            // arm_model_group = arm_move_group.getCurrentState()->getJointModelGroup(planning_group);
            pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);
            execution_time_service = n.advertiseService("/get_execution_time", &ManipulatorArm::executionTime, this);
        }

        bool executionTime(manipulator_control::TrajectoryExecution::Request &req,
                           manipulator_control::TrajectoryExecution::Response &res) {
            
            arm_move_group.setPoseTarget(req.pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            res.duration = arm_move_group.getPlanningTime();
            return true;
        }

        void main_loop(void) {
            ros::Rate loop_rate(100);
            ros::AsyncSpinner spinner(1);
            spinner.start();

            // move group setup

            namespace rvt = rviz_visual_tools;
            moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

            visual_tools.deleteAllMarkers();

            Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
            text_pose.translation().z() = 1.0;
            visual_tools.publishText(text_pose, "Manipulator Motion Planning", rvt::WHITE, rvt::XLARGE);

            visual_tools.trigger();

            while(ros::ok()) {

            }

            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "manipulator_control");
    std::string ARM_PLANNING_GROUP = "arm";
    std::string base_link = "base_link";
    ManipulatorArm node = ManipulatorArm(ARM_PLANNING_GROUP, base_link);
    node.main_loop();
    return 0;
}