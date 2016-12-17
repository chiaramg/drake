#include <example_chiara_message/exmpl_chiara.h>

#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

#include <ros/ros.h>

//void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)
void firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)

{
    std::cout << "This is the listener, Message received from drake" << std::endl;

    std::vector<float> temp = my_array->data;

    Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 4, 12);

    std::cout << my_mat << std::endl;
}

messages_chiara::~messages_chiara(){}

int main(int argc, char **argv) {

    //Subscribe to talker:
    ros::init(argc, argv, "listen");

    ros::NodeHandle n;

    ros::Subscriber sub;
    sub = n.subscribe("chatter2", 10, firstCallback);

    //messages_chiara firstMessage(n);  // firstMessage.sub, firstMessage.pub, firstMessage::firstCallback

    ros::spin();

    return 0;
}