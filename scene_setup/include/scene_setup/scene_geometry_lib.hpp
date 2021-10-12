#ifndef SCENE_GEOMETRY_INCLUDE_GUARD_HPP
#define SCENE_GEOMETRY_INCLUDE_GUARD_HPP

#include <vector>
#include <cmath>
#include <iostream>

namespace scene
{
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
            /// \param dims_f;
            /// \param dist_f;
            /// \param dist_r;
            Scene(const std::vector<double> &dims_f, double dist_f, double dist_r);

            /// \brief inserts a rectangular object into the scene
            
            void insertObject(const std::vector<double> dims, std::vector<double> front_plane_pos);
    };
}
#endif
