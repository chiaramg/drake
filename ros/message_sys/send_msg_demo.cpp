#include <example_chiara_message/exmpl_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

#include <ros/ros.h>



int main(int argc, char **argv) {

    ros::init(argc, argv, "publishMatrix");

    ros::NodeHandle n;

    ros::Publisher pub;
    pub = n.advertise<std_msgs::Float32MultiArray>("chatter1", 100);
    ros::Rate loop_rate(10);

    // create eigen::matrix

    Eigen::VectorXf vec(12);

    for (int i = 0; i < 12; i++) {
        vec(i) = 1.0;
    }

    std_msgs::Float32MultiArray my_array;

    long double ctr = -1;
    while (ros::ok()) {

        my_array.data.clear();
        for (int i = 0; i < 12; i++) {
           vec(i) += ++ctr;
            my_array.data.push_back(vec(i));
        }

        //firstMessage.pub.publish(my_array);
        pub.publish(my_array);
        std::cout<<"here's what i send to Drake:" << vec << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}