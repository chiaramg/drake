#include "include/msgs_chiara.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>



messages_chiara::messages_chiara(ros::NodeHandle& nh_)

{
    //SUBSCRIBERS
    sub = nh_.subscribe("chat2", 100, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    pub = nh_.advertise<std_msgs::Float32MultiArray>("chatter", 1);

}

messages_chiara::~messages_chiara(){}

void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)
{
    std::cout << "Message received" << std::endl;

    std::vector<float> temp = my_array->data;

    Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);

    std::cout << my_mat << std::endl;

}
