#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <shape_msgs/SolidPrimitive.h>

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
#include "manipulator_control/RemoveObject.h"

#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

static constexpr double PI=3.14159265358979323846;

class ManipulatorArm {
    private:
        // ros variables
        ros::NodeHandle n;
        ros::Publisher pincer_pub;
        ros::ServiceServer execution_time_service;
        ros::ServiceServer remove_object_service;
        ros::Subscriber object_sub;

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
            remove_object_service = n.advertiseService("/remove_object", &ManipulatorArm::removeObject, this);
            object_sub = n.subscribe("objects", 10, &ManipulatorArm::object_callback, this);
        }

        bool executionTime(manipulator_control::TrajectoryExecution::Request &req,
                           manipulator_control::TrajectoryExecution::Response &res) {
            
            geometry_msgs::Pose grab_pose;
            grab_pose.position.x = req.block.pose.position.x;
            grab_pose.position.y = req.block.pose.position.y;
            grab_pose.position.z = req.block.pose.position.z + 0.1;
            
            tf2::Quaternion grab_quat;
            grab_quat.setRPY(PI/2, PI/2, 0.0);
            grab_pose.orientation = tf2::toMsg(grab_quat);
            
            arm_move_group.setPoseTarget(grab_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                // store the plan
                arm_move_group.plan(plan);

                moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;

                trajectory_msgs::JointTrajectory joint_traj = trajectory.joint_trajectory;

                trajectory_msgs::JointTrajectoryPoint last_point = joint_traj.points[joint_traj.points.size()-1];
                
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
            quat.setRPY(PI/2, PI/2, 0.0);
            pose.orientation = tf2::toMsg(quat);
            pose.position.x = req.block.pose.position.x;
            pose.position.y = req.block.pose.position.y;
            pose.position.z = req.block.pose.position.z + 0.075;

            arm_move_group.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }

            std_msgs::Float64 pincer_angle;
            pincer_angle.data = 0.90;
            pincer_pub.publish(pincer_angle);

            // move the arm to the grasp pose
            pose.position.z = req.block.pose.position.z - 0.02;
            arm_move_group.setPoseTarget(pose);
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }

            pincer_angle.data = 0.0;
            pincer_pub.publish(pincer_angle);

            // TODO: attach collision object to arm!!!

            pose.position.x = req.block.pose.position.y;
            pose.position.y = req.block.pose.position.x;
            pose.position.z = req.block.pose.position.z + 0.075;

            arm_move_group.setPoseTarget(pose);
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }

            pose.position.z = req.block.pose.position.z - 0.02;
            arm_move_group.setPoseTarget(pose);
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }

            pincer_angle.data = 0.90;
            pincer_pub.publish(pincer_angle);
            return true;
        }
        
        void object_callback(const scene_setup::BlockArray msg) {
            
            std::vector<moveit_msgs::CollisionObject> collision_objects;

            // loop through all blocks received and add them as collision objects to the MoveIt planner
            for (auto block : msg.blocks) {
                std::cout << "object array received!!++++++++++++++++++++++++++++++++++++++++++ \r" << std::endl;


                moveit_msgs::CollisionObject object;
                object.id = "block" + std::to_string(block.id);
                std::cout << object.id << "\r" << std::endl;
                object.header.frame_id = "base_link";
                
                // define a box to add to the world
                shape_msgs::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = block.dimensions[0];
                primitive.dimensions[primitive.BOX_Y] = block.dimensions[1];
                primitive.dimensions[primitive.BOX_Z] = block.dimensions[2];

                // define a pose for the box
                geometry_msgs::Pose box_pose;
                box_pose.orientation.x = 1.0;
                box_pose.position.x = block.pose.position.x;
                box_pose.position.y = block.pose.position.y;
                box_pose.position.z = block.pose.position.z;

                object.primitives.push_back(primitive);
                object.primitive_poses.push_back(box_pose);
                object.operation = object.ADD;

                collision_objects.push_back(object);
            }

            planning_scene_interface.addCollisionObjects(collision_objects);
            return;
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