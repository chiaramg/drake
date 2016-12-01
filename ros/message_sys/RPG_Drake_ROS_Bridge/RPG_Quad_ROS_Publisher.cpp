//
// Created by chiara on 30.11.16.
//

#include "drake/examples_chiara/message_sys/RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


namespace drake {
namespace systems{
template <typename T>
RPG_quad_ROS_publisher<T>::RPG_quad_ROS_publisher():


template <typename T>
void RPG_quad_ROS_publisher<T>::EvalOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
    DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

    ros::init(argc, argv, "publishVector");
    ros::NodeHandle n;
    messages_chiara firstMessage(n); // firstMessage.sub, firstMessage.pub, firstMessage.firstCallback
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


}


}   // namespace systems
}   // namespace drake