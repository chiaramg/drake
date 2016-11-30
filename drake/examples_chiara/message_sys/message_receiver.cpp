//
// Created by chiara on 29.11.16.
//


#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& my_vec){


    std::cout << "Message received" << std::endl;
    std::vector<float> temp = my_vec->data;

    Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);

    std::cout << my_mat << std::endl;

}

int main(int argc, char **argv) {

    //Subscribe to talker:
    ros::init(argc, argv, "listen");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 100, chatterCallback);

    ros::spin();

    return 0;
}