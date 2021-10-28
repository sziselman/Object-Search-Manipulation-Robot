#include "scene_setup/scene_geometry_lib.hpp"

namespace scene
{
    CubeFace::CubeFace()
    {
        geometry_msgs::Point point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        lower_right = point;
        lower_left = point;
        upper_right = point;
        upper_left = point;
    }

    CubeFace::CubeFace(geometry_msgs::Point lr, geometry_msgs::Point ll, geometry_msgs::Point ur, geometry_msgs::Point ul)
    {
        lower_right = lr;
        lower_left = ll;
        upper_right = ur;
        upper_left = ul;
    }

    CubeFace CubeFace::getShadow(double distance)
    {
        CubeFace shadow;
        shadow.lower_right.x = (lower_right.x / lower_right.y) * distance;
        shadow.lower_right.y = distance;
        shadow.lower_right.z = (lower_right.z / lower_right.y) * distance;

        shadow.upper_right.x = (upper_right.x / upper_right.y) * distance;
        shadow.upper_right.y = distance;
        shadow.upper_right.z = (upper_right.z / upper_right.y) * distance;

        shadow.lower_left.x = (lower_left.x / lower_left.y) * distance;
        shadow.lower_left.y = distance;
        shadow.lower_left.z = (lower_left.z / lower_left.y) * distance;

        shadow.upper_left.x = (upper_left.x / upper_left.y) * distance;
        shadow.upper_left.y = distance;
        shadow.upper_left.z = (upper_left.z / upper_left.y) * distance;

        return shadow;
    }

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
        cout << "volume of scene is " << volume_scene << " m^2" << endl;
    }

    double Scene::getObjectVisibility(const std::vector<double> dims, geometry_msgs::Pose pose)
    {
        using namespace std;

        // area of the front face of the object
        double object_front_area = dims[0] * dims[1];
        std::cout << "area of front plane of object is " << object_front_area << endl;

        // volume of the object
        double object_volume = dims[0] * dims[1] * dims[2];
        std::cout << "volume of the object is " << object_volume << endl;

        // distance between the front face of the object and the rear plane of the scene
        double h = rear_dist - pose.position.y + (dims[1] / 2);
        std::cout << "distance between two planes of frustum is " << h << endl;

        CubeFace object_face;
        object_face.lower_right.x = pose.position.x + (dims[0] / 2);
        object_face.lower_right.y = pose.position.y - (dims[1] / 2);
        object_face.lower_right.z = pose.position.z - (dims[2] / 2);

        object_face.lower_left.x = pose.position.x - (dims[0] / 2);
        object_face.lower_left.y = pose.position.y - (dims[1] / 2);
        object_face.lower_left.z = pose.position.z - (dims[2] / 2);

        object_face.upper_right.x = pose.position.x + (dims[0] / 2);
        object_face.upper_right.y = pose.position.y - (dims[1] / 2);
        object_face.upper_right.z = pose.position.z + (dims[2] / 2);

        object_face.upper_left.x = pose.position.x - (dims[0] / 2);
        object_face.upper_left.y = pose.position.y - (dims[1] / 2);
        object_face.upper_left.z = pose.position.z + (dims[2] / 2);

        CubeFace shadow_face = object_face.getShadow(rear_dist);

        double shadow_area = fabs(shadow_face.lower_right.x - shadow_face.lower_left.x) * fabs(shadow_face.upper_right.z - shadow_face.lower_right.z);

        std::cout << "area of shadow is " << shadow_area << endl;
        
        return ((h / 3) * (object_front_area + shadow_area + sqrt(object_front_area * shadow_area))) - object_volume;
    }
}