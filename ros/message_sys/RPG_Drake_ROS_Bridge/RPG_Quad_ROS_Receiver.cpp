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

template<typename T>
RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver() {
    this->DeclareInputPort(kVectorValued, 1, kContinuousSampling);
    this->DeclareOutputPort(kVectorValued, 1, kContinuousSampling);
}

template<typename T>
RPG_quad_ROS_receiver<T>::~RPG_quad_ROS_receiver() {}

template<typename T>
const SystemPortDescriptor<T>&
RPG_quad_ROS_receiver<T>::get_input_port() const {
    return System<T>::get_input_port(0);
}

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

    std::cout<<"In the receiver"<<std::endl;

    ros::NodeHandle n;
    messages_chiara receiveMessage(n);


    //ros::spin();
}

template class RPG_quad_ROS_receiver<double>;
}   // namespace systems
}   // namespace drake


