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

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

#include "manipulator_control/TrajectoryExecution.h"


static constexpr double PI=3.14159265358979323846;

class ManipulatorArm {
    private:
        // ros variables
        ros::NodeHandle n;
        ros::Publisher pincer_pub;
        ros::ServiceServer execution_time_service;
        ros::ServiceServer remove_object_service;
        ros::Publisher pincer_pub;

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
            remove_object_service = n.advertiseService("/remove_object", &Manipulator::removeObject, this);
            pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);
        }

        bool executionTime(manipulator_control::TrajectoryExecution::Request &req,
                           manipulator_control::TrajectoryExecution::Response &res) {
            
            geometry_msgs::Pose grab_pose;
            grab_pose.position.x = req.pose.position.x;
            grab_pose.position.y = req.pose.position.y;
            grab_pose.position.z = req.pose.position.z + 0.05;
            
            tf2::Quaternion grab_quat;
            grab_quat.setRPY(PI/2, PI/2, 0.0);
            grab_pose.orientation = tf2::toMsg(grab_quat);
            
            arm_move_group.setPoseTarget(grab_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                // store the plan
                arm_move_group.plan(plan);

                // std::cout << "planned trajectory " << std::endl;

                moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
                trajectory_msgs::JointTrajectory joint_traj = trajectory.joint_trajectory;

                // std::cout << "joint trajectory\r" << std::endl;

                trajectory_msgs::JointTrajectoryPoint last_point = joint_traj.points[joint_traj.points.size()-1];
                
                // std::cout << "execution time " << last_point.time_from_start << "\r" << std::endl;
                
                res.success = true;
                res.duration = last_point.time_from_start.toSec();
            }
            else {
                res.success = false;
                res.duration = 1e5;
            }
            
            return true;
        }

        bool removeObject(manipulator_control::RemoveObject::Request &req,
                          manipulator_control::RemoveObject::Response &res) {
            
            // move the arm to the pre grasp pose
            geometry_msgs::Pose pose;
            tf2::Quaternion quat;
            quat.setRPY(PI/2, PI/2 0);
            pose.orientation = tf2::toMsg(quat);
            pose.position.x = req.block.pose.position.x;
            pose.position.y = req.block.pose.position.y;
            pose.position.z = req.block.pose.position.z + 0.075;

            arm_move_group.setPoseTarget(pose);
            arm_move_group.move();

            // move the arm to the grasp pose
            pose.position.z = req.block.pose.position.z - 0.02;

            arm_move_group.setPoseTarget(pose);
            arm_move_group.move();
            return true;

            // close the grippers, attach brick to the move group
            std_msgs::Float64 angle;
            angle.data = 0.80;
            pincer_pub.publish(close_angle);
            // arm_move_group.attachObject("")


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
                ros::spinOnce();
                loop_rate.sleep();
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