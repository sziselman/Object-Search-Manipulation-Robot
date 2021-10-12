#include "scene_setup/scene_geometry_lib.hpp"

namespace scene
{
    void Scene::getSceneGeometry(void)
    {
        // calculates the dimensions of the rear plane
        rear_plane_dims.resize(2);
        rear_plane_dims[0] = (front_plane_dims[0] / front_dist) * rear_dist;
        rear_plane_dims[1] = (front_plane_dims[1] / front_dist) * rear_dist;

        // calculates the volume of the scene (frustum)
        double area_f = front_plane_dims[0] * front_plane_dims[1];
        double area_r = rear_plane_dims[0] * rear_plane_dims[1];
        double h = rear_dist - front_dist;
        volume_scene = (h/3) * (area_f + area_r + sqrt(area_f * area_r));
    }

    Scene::Scene()
    {
        front_plane_dims.resize(2);
        front_plane_dims[0] = 1.0;
        front_plane_dims[1] = 1.0;
        front_dist = 1.0;
        rear_dist = 2.0;

        getSceneGeometry();
    }

    Scene::Scene(const std::vector<double> &dims_f, double dist_f, double dist_r)
    {
        using namespace std;

        front_plane_dims = dims_f;
        front_dist = dist_f;
        rear_dist = dist_r;

        getSceneGeometry();

        cout << "rear plane dimensions are " << rear_plane_dims[0] << ", " << rear_plane_dims[1] << endl;
        cout << "volume of scene is " << volume_scene << " m^2" endl;
    }
}