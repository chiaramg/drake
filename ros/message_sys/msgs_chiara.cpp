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

/*
int main(int argc, char **argv) {

    ros::init(argc, argv, "publishMatrix");

    ros::NodeHandle n;

    messages_chiara firstMessage(n);  // firstMessage.sub, firstMessage.pub, firstMessage::firstCallback

    ros::Rate loop_rate(10);

    // create eigen::matrix

    Eigen::MatrixXf eig_matrix(6, 5);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            eig_matrix(i, j) = i * 5 + j;
        }
    }

    std_msgs::Float32MultiArray my_array;

    my_array.data.clear();

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            my_array.data.push_back(eig_matrix(i, j));
        }
    }


    while (ros::ok()) {

        firstMessage.pub.publish(my_array);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}*/