#include <ros/ros.h>

class ObjectHandler {
    private:
        // pubs, subs, servs, etc.
        ros::NodeHandle n;
        ros::Publisher object_pub;

        // parameters
        std::vector<double> object_dimensions;
        std::vector<double> object1_position;
        std::vector<double> object2_position;
        std::vector<double> object3_position;

    public:
        ObjectHandler() {

        }

        void main_loop(void) {
            ros::Rate loop_rate(100);

            while (ros::ok()) {
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