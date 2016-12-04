#include "RPG_Quad_ROS_Receiver.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>



namespace drake {
namespace systems {

//template <typename T>
//RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver();

/*

messages_chiara(ros::NodeHandle& nh_) {

    //SUBSCRIBERS
    sub = nh_.subscribe("chatter", 1, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    //pub = nh_.advertise<std_msgs::Float32MultiArray>("chat2", 100);
}

*/

template <typename T>
const SystemPortDescriptor<T>&
RPG_quad_ROS_receiver<T>::get_output_port() const {
    return System<T>::get_output_port(0);
}

template <typename T>
void RPG_quad_ROS_receiver<T>::EvalOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
    DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

    ros::NodeHandle n;
    messages_chiara firstMessage(n);


    //ros::spin();

}

}   // namespace systems
}   // namespace drake


