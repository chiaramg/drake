#include <example_chiara_message/exmpl_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

#include <ros/ros.h>

/*
messages_chiara::messages_chiara(ros::NodeHandle& nh_)

{
    //SUBSCRIBERS
    sub = nh_.subscribe("chat2", 100, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    pub = nh_.advertise<std_msgs::Float32MultiArray>("chatter1", 100);

}
*/
/*
messages_chiara::~messages_chiara(){}

void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)
{
    std::cout << "This is the initial talker, Message received" << std::endl;

    std::vector<float> temp = my_array->data;

    Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);

    std::cout << my_mat << std::endl;

}*/


int main(int argc, char **argv) {

    ros::init(argc, argv, "publishMatrix");

    ros::NodeHandle n;

//    messages_chiara firstMessage(n);  // firstMessage.sub, firstMessage.pub, firstMessage::firstCallback
    ros::Publisher pub;
    pub = n.advertise<std_msgs::Float32MultiArray>("chatter1", 100);
    ros::Rate loop_rate(10);

    // create eigen::matrix

    Eigen::VectorXf vec(12);

    for (int i = 0; i < 12; i++) {
        vec(i) = 1.0;
    }

    std_msgs::Float32MultiArray my_array;

    my_array.data.clear();

    for (int i = 0; i < 12; i++) {
            my_array.data.push_back(vec(i));
    }

    while (ros::ok()) {

        //firstMessage.pub.publish(my_array);
        pub.publish(my_array);
        std::cout<<"here's what i send to Drake:" << vec << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}