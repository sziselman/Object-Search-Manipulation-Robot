#ifndef SCENE_GEOMETRY_INCLUDE_GUARD_HPP
#define SCENE_GEOMETRY_INCLUDE_GUARD_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include "scene_setup/Block.h"

namespace scene
{
    struct ObjectFacePoints
    {
        std::vector<geometry_msgs::Point> front_face_points;
        std::vector<geometry_msgs::Point> foot_print_points;

        ObjectFacePoints();

        ObjectFacePoints(scene_setup::Block block);

        double getShadowArea(double y_shadow);
    };

    class Scene
    {
        private:
            std::vector<double> front_plane_dims;       // the 2D dimensions that confine the front plane
            std::vector<double> rear_plane_dims;        // the 2D dimensions that confine the rear plane
            double front_dist;                          // the distance between the "camera" and the front plane
            double rear_dist;                           // the distance between the "camera" and the rear plane
            double volume_scene;                        // the volume of the scene

            void getSceneGeometry(void);

        public:
            /// \brief constructor for Scene object
            Scene();

            /// \brief create a scene given dimensions and distance
            /// \param dims_f
            /// \param dist_f
            /// \param dist_r
            Scene(std::vector<double> dims_f, double dist_f, double dist_r);

            /// \brief inserts a rectangular object into the scene
            /// \param block : stores the position, dimensions, id and utility
            /// \returns the volume of the space that is blocked by the object
            double getObjectVisibility(scene_setup::Block block);

            std::vector<double> getRearPlaneDimensions(void);
    };
}
#endif
