#include <ros/ros.h>
#include <map>
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "scene_setup/Block.h"
#include "scene_setup/BlockArray.h"

#include "scene_setup/RemoveObjectId.h"

class ObjectHandler {
    private:
        // pubs, subs, servs, etc.
        ros::NodeHandle n;

        ros::Publisher object_pub;
        ros::Publisher object_marker_pub;
        ros::Publisher target_marker_pub;

        ros::ServiceServer remove_object_id_service;

        // parameters
        std::vector<double> lar_obj_dims;
        std::vector<double> med_obj_dims;
        std::vector<double> sma_obj_dims;

        std::vector<double> object_dimensions;

        std::vector<double> object1_position;
        std::vector<double> object2_position;
        std::vector<double> object3_position;
        std::vector<double> object4_position;

        std::vector<double> goal_object_position;
        std::vector<std::vector<double>> object_positions;
        int frequency;

        // maps the block id to the block
        std::map<int, scene_setup::Block> object_map = {};

        scene_setup::BlockArray block_arr;
        visualization_msgs::MarkerArray marker_arr;

    public:
        /// \brief the constructor for the ObjectHandler object
        ObjectHandler() {
            load_parameters();

            object_pub = n.advertise<scene_setup::BlockArray>("objects", 10, true);
            target_marker_pub = n.advertise<visualization_msgs::Marker>("target_marker", 10, true);
            object_marker_pub = n.advertise<visualization_msgs::MarkerArray>("object_markers", 10, true);

            remove_object_id_service = n.advertiseService("remove_object_id", &ObjectHandler::remove_object_id, this);
            
            initialize_dictionary();
            initialize_object_markers();
            initialize_target_marker();
        }

        /// \brief loads parameters from the parameter server
        void load_parameters(void) {
            n.getParam("object_dimensions", object_dimensions);

            n.getParam("lar_block_dimensions", lar_obj_dims);
            n.getParam("med_block_dimensions", med_obj_dims);
            n.getParam("sma_block_dimensions", sma_obj_dims);

            n.getParam("object1_position", object1_position);
            object_positions.push_back(object1_position);
            n.getParam("object2_position", object2_position);
            object_positions.push_back(object2_position);
            n.getParam("object3_position", object3_position);
            object_positions.push_back(object3_position);
            n.getParam("object4_position", object4_position);
            object_positions.push_back(object4_position);

            n.getParam("frequency", frequency);

            n.getParam("goal_object_position", goal_object_position);
        }

        /// \brief initializes the dictionary that keeps track of which objects are visible within the scene
        void initialize_dictionary(void) {
            int id = 1;

            std::vector<int> occludes_vec, occluded_by_vec;

            // large object (1)
            scene_setup::Block block;
            block.pose.position.x = object1_position[0];
            block.pose.position.y = object1_position[1];
            block.pose.position.z = object1_position[2];
            block.pose.orientation.w = 1.0;
            block.dimensions = lar_obj_dims;
            block.id = 1;
            block.goal = false;
            
            occludes_vec.push_back(4);
            block.occludes = occludes_vec;

            object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));

            // medium object (2)
            block.pose.position.x = object2_position[0];
            block.pose.position.y = object2_position[1];
            block.pose.position.z = object2_position[2];
            block.dimensions = med_obj_dims;
            block.id = 2;

            occludes_vec.clear();
            block.occludes = occludes_vec;

            object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));

            // medium object (3)
            block.pose.position.x = object3_position[0];
            block.pose.position.y = object3_position[1];
            block.pose.position.z = object3_position[2];
            block.id = 3;
            
            object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));

            // medium object (4)
            block.pose.position.x = object4_position[0];
            block.pose.position.y = object4_position[1];
            block.pose.position.z = object4_position[2];
            block.id = 4;

            occluded_by_vec.push_back(1);
            block.occluded_by = occluded_by_vec;

            occludes_vec.clear();
            occluded_by_vec.clear();

            object_map.insert(std::pair<int, scene_setup::Block>(block.id, block));

            return;
        }

        /// \brief removes the object with user-specified id from the scene
        /// \param req - (scene_setup/RemoveObjectId/Request) contains an integer that represents the id of the object to be removed
        /// \param res - (scene_setup/RemoveObjectId/Response) none
        /// \return the boolean value true (if service is success) or false (if service fails)
        bool remove_object_id(scene_setup::RemoveObjectId::Request &req,
                              scene_setup::RemoveObjectId::Response &res) {
            
            std::cout << "erasing object of id " << req.id << "\r" << std::endl;
            object_map.erase(req.id);

            visualization_msgs::Marker delete_all_marker;
            delete_all_marker.header.frame_id = "base_link";
            delete_all_marker.action = 3;
            marker_arr.markers.clear();

            marker_arr.markers.push_back(delete_all_marker);
            
            object_marker_pub.publish(marker_arr);
            
            marker_arr.markers.clear();

            initialize_object_markers();

            return true;
        }

        /// \brief publishes the marker that represents the target object
        void initialize_target_marker(void) {

            visualization_msgs::Marker target;
            target.header.frame_id = "base_link";
            target.ns = "target";
            target.type = 1;
            target.action = 0;
            target.id = 0;
            target.pose.position.x = goal_object_position[0];
            target.pose.position.y = goal_object_position[1];
            target.pose.position.z = goal_object_position[2];
            target.scale.x = sma_obj_dims[0];
            target.scale.y = sma_obj_dims[1];
            target.scale.z = sma_obj_dims[2];
            target.color.a = 1.0;
            target.color.r = 202./255.;
            target.color.g = 231./255.;
            target.color.b = 193./255.;

            target_marker_pub.publish(target);

            return;
        }

        /// \brief publishes the markers that represent the objects within the scene (not including the target object)
        void initialize_object_markers(void) {
        
            visualization_msgs::Marker obj;

            obj.header.frame_id = "base_link";
            obj.ns = "objects";
            obj.type = 1;
            obj.action = 0;

            // loop through the initialized dictionary and publish the markers
            for (auto const& val : object_map) {

                obj.id = val.second.id;

                obj.pose = val.second.pose;
                obj.scale.x = val.second.dimensions[0];
                obj.scale.y = val.second.dimensions[1];
                obj.scale.z = val.second.dimensions[2];
                obj.color.a = 1.0;
                obj.color.r = 250./255.;
                obj.color.g = 218./255.;
                obj.color.b = 221./255.;

                marker_arr.markers.push_back(obj);
            }

            object_marker_pub.publish(marker_arr);

            return;
        }

        /// \brief the main loop to be executed
        void main_loop(void) {
            ros::Rate loop_rate(100);

            while (ros::ok()) {

                for (auto const& val : object_map) {
                    block_arr.blocks.push_back(val.second);
                }
                
                object_pub.publish(block_arr);

                block_arr.blocks.clear();
                
                ros::spinOnce();
                loop_rate.sleep();
            }
            return;
        }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "object_handler");
    ObjectHandler node;
    node.main_loop();
    return 0;
}