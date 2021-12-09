#include <ros/ros.h>
#include <ros/console.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <shape_msgs/SolidPrimitive.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
#include "scene_setup/RemoveObjectId.h"

static constexpr double PI=3.14159265358979323846;

class ManipulatorArm {
    private:
        // ros variables
        ros::NodeHandle n;

        ros::Publisher pincer_pub;

        ros::ServiceServer execution_time_service;
        ros::ServiceServer remove_object_service;

        ros::ServiceClient remove_object_id_client;

        ros::Subscriber object_sub;

        // move it variables
        moveit::planning_interface::MoveGroupInterface arm_move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const moveit::core::JointModelGroup* arm_model_group;

        moveit_visual_tools::MoveItVisualTools visual_tools;

        // variables
        visualization_msgs::MarkerArray object_arr;
        std::vector<scene_setup::Block> blocks;
        std::vector<std::string> object_ids;

    public:
        /// \brief constructor for ManipulatorArm object
        /// \param planning_group - the string that contains the name of the arm's move group
        /// \param frame - the string that is the name of the base_link frame
        ManipulatorArm(std::string planning_group, std::string frame) : arm_move_group(planning_group),
                                                                        visual_tools(frame) {

            pincer_pub = n.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);

            execution_time_service = n.advertiseService("get_execution_time", &ManipulatorArm::execution_time, this);
            remove_object_service = n.advertiseService("remove_object", &ManipulatorArm::remove_object, this);

            remove_object_id_client = n.serviceClient<scene_setup::RemoveObjectId>("remove_object_id");

            object_sub = n.subscribe("objects", 10, &ManipulatorArm::object_callback, this);
        }

        /// \brief callback function for subscriber to the /objects topic to read in the objects that are seen
        void object_callback(const scene_setup::BlockArray msg) {
            blocks = msg.blocks;
            return;
        }

        /// \brief function for /get_execution_time service, calculates the time to execute a trajectory to move a user-specified block
        /// \param req - (manipulator_control/TrajectoryExecution/Request) contains a block that is to be removed from the scene
        /// \param res - (manipulator_control/TrajectoryExecution/Response) contains a float (the duration of the trajectory) and a boolean (indicates if the trajectory was successful or not)
        /// \return the boolean value true (if service is successful) or false (if the service fails)
        bool execution_time(manipulator_control::TrajectoryExecution::Request &req,
                            manipulator_control::TrajectoryExecution::Response &res) {
            
            geometry_msgs::Pose grab_pose;
            grab_pose.position.x = req.block.pose.position.x;
            grab_pose.position.y = req.block.pose.position.y;
            grab_pose.position.z = req.block.pose.position.z + 0.1;
            
            tf2::Quaternion grab_quat;
            grab_quat.setRPY(0.0, PI/2, 0.0);
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

        /// \brief function that adds all other objects in the scene (except the one being removed) as collision objects
        /// \param id - the id of the block being removed
        void add_collision_objects(int id) {
            
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            object_ids.clear();

            // loop through the blocks
            for (auto block : blocks) {

                // if the id is not the one being removed, add it as a collision object
                if (block.id != id) {
                    moveit_msgs::CollisionObject obj;
                    obj.header.frame_id = arm_move_group.getPlanningFrame();
                    obj.id = std::to_string(block.id);
                    object_ids.push_back(obj.id);

                    shape_msgs::SolidPrimitive prim;
                    prim.type = prim.BOX;
                    prim.dimensions.resize(3);
                    for (int i = 0; i < block.dimensions.size(); i++) {
                        prim.dimensions[i] = block.dimensions[i];
                    }

                    obj.primitives.push_back(prim);
                    obj.primitive_poses.push_back(block.pose);
                    obj.operation = obj.ADD;

                    collision_objects.push_back(obj);
                }
            }

            planning_scene_interface.addCollisionObjects(collision_objects);

            return;
        }

        /// \brief function for /remove_object service, commands the adroit manipulator arm to remove an object from the scene
        /// \param req - (manipulator_control/RemoveObject/Request) contains a block that is to be removed from the scene
        /// \param res - (manipulator_control/RemoveObject/Response) contains a boolean (indicates if the removal of the object was successful or not)
        /// \return the boolean value true (if service is successful) or false (if the service fails)
        bool remove_object(manipulator_control::RemoveObject::Request &req,
                           manipulator_control::RemoveObject::Response &res) {
            
            // add collision objects for all objects except the one being removed
            add_collision_objects(req.block.id);

            std_msgs::Float64 pincer_angle;
            pincer_angle.data = 0.90;
            pincer_pub.publish(pincer_angle);

            // move the arm to the pre-grasp pose
            geometry_msgs::Pose pose;
            tf2::Quaternion quat;
            quat.setRPY(0.0, PI/2, 0.0);
            pose.orientation = tf2::toMsg(quat);
            pose.position.x = req.block.pose.position.x;
            pose.position.y = req.block.pose.position.y;
            pose.position.z = req.block.pose.position.z + 0.070;

            arm_move_group.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }
            else {
                return false;
            }

            // move the arm to the grasp pose
            pose.position.z = req.block.pose.position.z + 0.025;

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pose);

            moveit_msgs::RobotTrajectory traj;

            const double jump_thresh = 0.0;
            const double eef_step = 0.01;
            double fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

            if (fraction == 1.0) {
                arm_move_group.execute(traj);
            }
            else {
                std::cout << "planning failed :( \r" << std::endl;
                return false;
            }
            

            pincer_angle.data = 0.0;
            pincer_pub.publish(pincer_angle);

            // move arm back to pre grasp pose
            pose.position.z = req.block.pose.position.z + 0.070;

            waypoints.clear();
            waypoints.push_back(pose);

            fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

            if (fraction == 1.0) {
                arm_move_group.execute(traj);
            }
            else {
                std::cout << "planning failed >:( \r" << std::endl;
                return false;
            }

            // move arm to pre release pose
            quat.setRPY(PI/2, PI/2, 0.0);
            pose.orientation = tf2::toMsg(quat);
            pose.position.x = 0.3;
            pose.position.y = 0.0;

            // // add orientation constraint
            // moveit_msgs::OrientationConstraint ocm;
            // ocm.header.frame_id = "base_link";
            // ocm.link_name = "endpoint_link";
            // ocm.orientation = tf2::toMsg(quat);


            // // add positional constraint
            // moveit_msgs::PositionConstraint pcm;
            // pcm.header.frame_id = "base_link";
            // pcm.link_name = "endpoint_link";
            // pcm.target_point_offset[2] = 0.0;

            arm_move_group.setPoseTarget(pose);
            if (arm_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                arm_move_group.move();
            }
            else {
                return false;
            }

            // release the grippers
            pincer_angle.data = 0.90;
            pincer_pub.publish(pincer_angle);

            pose.position.z = req.block.pose.position.z + 0.070;
            waypoints.clear();
            waypoints.push_back(pose);
            fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

            if (fraction == 1.0) {
                arm_move_group.execute(traj);
            }
            else {
                std::cout << "planning failed D:< \r" << std::endl;
                return false;
            }

            pincer_angle.data = 0.0;
            pincer_pub.publish(pincer_angle);

            // remove the object from the scene by updating object_handler node
            scene_setup::RemoveObjectId msg;
            msg.request.id = req.block.id;

            remove_object_id_client.call(msg);

            // remove all the collision objects from the world
            planning_scene_interface.removeCollisionObjects(object_ids);

            return true;
        }

        /// \brief the main loop to be executed
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