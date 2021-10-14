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

#include "control/control_library.hpp"


void objectCallback(const geometry_msgs::Pose pose);

static geometry_msgs::Pose object_pose;

enum State {Idle, Stow, PreGraspPose, GraspPose, PrePlacePose, PlacePose, GrabObject, ReleaseObject};
static State current_state = Idle;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulator_trajectory");
    ros::NodeHandle n;

    ros::Subscriber object_sub = n.subscribe("/object", 10, objectCallback);

    ros::Rate loop_rate(100);
    std::vector<double> adroit_stow_positions{0.0, 1.5708, -1.5, 0.0, -0.0708, 1.5708};

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("beginning setup...");

    // Planning Groups
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string PINCER_PLANNING_GROUP = "pincer";

    // Set up the Move Group Interface using the planning group
    moveit::planning_interface::MoveGroupInterface arm_move_group_interface(ARM_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface pincer_move_group_interface(PINCER_PLANNING_GROUP);

    // Add collision objects in virtual world scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers used to refer to the planning group
    const moveit::core::JointModelGroup* arm_model_group = arm_move_group_interface.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    const moveit::core::JointModelGroup* pincer_model_group = pincer_move_group_interface.getCurrentState()->getJointModelGroup(PINCER_PLANNING_GROUP);

    ROS_INFO_STREAM("setup complete!!");

    ROS_INFO_STREAM("beginning visualization...");

    // provides capabilities for visualizing objects, robots and trajectories in RViz
    namespace rvt = rviz_visual_tools;

    // ARE THESE THE CORRECT JOINT NAMES WHEN VISUALIZING THE ARM AND PINCER??
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_STREAM("visualization complete!!");

    ROS_INFO_STREAM("begin trajectory planning and execution...");

    // add collision object
    addCollisionObjects(planning_scene_interface, object_pose);

    while (ros::ok())
    {
        // switch (current_state)
        // {
        //     case Idle:
        //     {
        //         break;
        //     }
        //     case Stow:
        //     {
        //         ROS_INFO_STREAM("moving adroit to stow position...");
        //         stowPosition(arm_move_group_interface, arm_model_group, adroit_stow_positions);
        //         current_state = Idle;
        //         break;
        //     }
        //     case PreGraspPose:
        //     {
        //         ROS_INFO_STREAM("moving pincer to pre-grasp position...");
        //         geometry_msgs::Pose pre_grasp_pose;
        //         tf2::Quaternion pre_grasp_quat;
        //         pre_grasp_quat.setRPY(PI/2, PI/2, )
        //     }
        // }
    }

}

void objectCallback(const geometry_msgs::Pose pose)
{
    object_pose = pose;
}