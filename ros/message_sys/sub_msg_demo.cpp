#include <example_chiara_message/exmpl_chiara.h>

#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

#include <ros/ros.h>
/*
messages_chiara::messages_chiara(ros::NodeHandle& nh_)
{
    //SUBSCRIBERS
    sub = nh_.subscribe("chatter2", 1, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    pub = nh_.advertise<std_msgs::Float32MultiArray>("chat2", 100);
}
*/
//void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)
void firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)

{
    std::cout << "This is the listener, Message received from drake" << std::endl;

    std::vector<float> temp = my_array->data;

    Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 4, 12);

    std::cout << my_mat << std::endl;


    //pub.publish(my_array);

    //std::cout << "sent the same matrix back" << std::endl;

}

messages_chiara::~messages_chiara(){}

int main(int argc, char **argv) {

    //Subscribe to talker:
    ros::init(argc, argv, "listen");

    ros::NodeHandle n;

    ros::Subscriber sub;
    sub = n.subscribe("chatter2", 1, firstCallback);

    //messages_chiara firstMessage(n);  // firstMessage.sub, firstMessage.pub, firstMessage::firstCallback

    ros::spin();

    return 0;
}