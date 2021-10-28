#ifndef SCENE_GEOMETRY_INCLUDE_GUARD_HPP
#define SCENE_GEOMETRY_INCLUDE_GUARD_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Pose.h>

namespace scene
{
    struct CubeFace
    {
        geometry_msgs::Point lower_right;
        geometry_msgs::Point lower_left;
        geometry_msgs::Point upper_right;
        geometry_msgs::Point upper_left;

        CubeFace();

        CubeFace(geometry_msgs::Point lr, geometry_msgs::Point ll, geometry_msgs::Point ur, geometry_msgs::Point ul);

        CubeFace getShadow(double distance);
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
            /// \brief creates a scene with 1x1 f
            Scene();

            /// \brief create a scene given dimensions and distance
            /// \param dims_f
            /// \param dist_f
            /// \param dist_r
            Scene(std::vector<double> dims_f, double dist_f, double dist_r);

            /// \brief inserts a rectangular object into the scene
            /// \param dims dimensions of the object
            /// \param pos position of the object
            /// \returns the volume of the space that is blocked by the object
            double getObjectVisibility(const std::vector<double> dims, geometry_msgs::Pose pose);
    };
}
#endif
