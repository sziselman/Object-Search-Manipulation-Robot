#include "scene_setup/scene_geometry_lib.hpp"

namespace scene
{
    void Scene::get_scene_geometry(void)
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

    ObjectFacePoints::ObjectFacePoints() {}

    ObjectFacePoints::ObjectFacePoints(scene_setup::Block block) {
        double x = block.pose.position.x;
        double y = block.pose.position.y;
        double z = block.pose.position.z;

        double w = block.dimensions[0];
        double d = block.dimensions[1];
        double h = block.dimensions[2];

        /*************************
         * Front face points
         * **********************/
        geometry_msgs::Point lower_left, lower_right, upper_right, upper_left;

        lower_left.x = x - (w/2);
        lower_left.y = y - (d/2);
        lower_left.z = z - (h/2);

        // std::cout << "lower left " << lower_left.x << ", " << lower_left.y << ", " << lower_left.z << "\r" << std::endl;

        front_face_points.push_back(lower_left);

        lower_right.x = x + (w/2);
        lower_right.y = y - (d/2);
        lower_right.z = z - (h/2);

        // std::cout << "lower right " << lower_right.x << ", " << lower_right.y << ", " << lower_right.z << "\r" << std::endl;

        front_face_points.push_back(lower_right);

        upper_right.x = x + (w/2);
        upper_right.y = y - (d/2);
        upper_right.z = z + (h/2);

        // std::cout << "upper_right " << upper_right.x << ", " << upper_right.y << ", " << upper_right.z << "\r" << std::endl;

        front_face_points.push_back(upper_right);

        upper_left.x = x - (w/2);
        upper_left.y = y - (d/2);
        upper_left.z = z + (h/2);

        // std::cout << "upper_left " << upper_left.x << ", " << upper_left.y << ", " << upper_left.z << "\r" << std::endl;

        front_face_points.push_back(upper_left);

        /*************************
         * Foot print points
         * **********************/
        geometry_msgs::Point back_left, back_right, front_right, front_left;

        back_left.x = x - (w/2);
        back_left.y = y + (d/2);

        foot_print_points.push_back(back_left);

        back_right.x = x + (w/2);
        back_right.y = y + (d/2);

        foot_print_points.push_back(back_right);

        front_right.x = x + (w/2);
        front_right.y = y - (d/2);

        foot_print_points.push_back(front_right);

        front_left.x = x - (w/2);
        front_left.y = y - (d/2);

        foot_print_points.push_back(front_left);
    }

    double ObjectFacePoints::get_shadow_area(double y_shadow) {
        // loop through the points and find the x and z values of the shadow
        std::vector<double> x_vals;
        std::vector<double> z_vals;

        for (auto p : foot_print_points) {
            double x = (p.x / p.y) * y_shadow;
            x_vals.push_back(x);
        }

        for (auto p : front_face_points) {
            double z = (p.z / p.y) * y_shadow;
            z_vals.push_back(z);
        }

        // sort the x and z values
        std::sort(x_vals.begin(), x_vals.end());
        std::sort(z_vals.begin(), z_vals.end());

        // get the width of shadow using minimum and maximum values
        double w_shadow = x_vals.back() - x_vals[0];
        // std::cout << "width of shadow is " << w_shadow << "\r" << std::endl;

        // get the height of shadow using minimum and maximum values
        double h_shadow = z_vals.back() - z_vals[0];
        // std::cout << "height of shadow is " << h_shadow << "\r" << std::endl;

        return w_shadow * h_shadow;
    }

    Scene::Scene() {}

    Scene::Scene(std::vector<double> dims_f, double dist_f, double dist_r) : front_plane_dims(dims_f), 
                                                                             front_dist(dist_f),
                                                                             rear_dist(dist_r) {
        get_scene_geometry();
    }

    double Scene::get_object_visibility(scene_setup::Block block)
    {
        std::cout << "getting occluded volume of block " << block.id << "+++++++++++++++++++\r" << std::endl;

        double x = block.pose.position.x;
        double y = block.pose.position.y;
        double z = block.pose.position.z;

        double w = block.dimensions[0];
        double d = block.dimensions[1];
        double h = block.dimensions[2];
        
        // area of the front face of the object
        double object_front_area = w * h;
        std::cout << "area of front plane of object is " << object_front_area << "\r" << std::endl;

        // volume of the object
        double object_volume = object_front_area * d;
        std::cout << "volume of the object is " << object_volume << "\r" << std::endl;

        // distance between the front face of the object and the rear plane of the scene
        double height = rear_dist - y + (d/2);
        std::cout << "distance between two planes of frustum is " << height << "\r" << std::endl;

        // initialize ObjectFacePoints object using the block
        ObjectFacePoints front_face(block);

        // get area of the shadow
        double shadow_area = front_face.get_shadow_area(rear_dist);

        return ((height / 3) * (object_front_area + shadow_area + sqrt(object_front_area * shadow_area))) - object_volume;
    }

    std::vector<double> Scene::get_rear_plane_dimensions(void) {
        return rear_plane_dims;
    }
}