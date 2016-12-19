#include "lqr_demo_subscriber.h"

#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <mutex>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace ros {
namespace message_sys {

template<typename T>
RPG_quad_ROS_subscriber<T>::RPG_quad_ROS_subscriber(
        const ros::NodeHandle& nh,
        const std::string& node_name, const int& data_dimension) :
        nh_(nh),
        data_dimension_(data_dimension),
        received_message_(Eigen::VectorXd::Zero(data_dimension)) {

    this->DeclareOutputPort(drake::systems::kVectorValued, data_dimension);

    // ROS Callback setup.
     sub_to_ros_ = nh_.subscribe(
             node_name, 100, &RPG_quad_ROS_subscriber<T>::TopicCallback, this);

    // start the async spinner.
    spinner.start();
}

template<typename T>
RPG_quad_ROS_subscriber<T>::~RPG_quad_ROS_subscriber() {}

template<typename T>
const drake::systems::SystemPortDescriptor<T>&
RPG_quad_ROS_subscriber<T>::get_output_port() const {
  return drake::systems::System<T>::get_output_port(0);
}

template<typename T>
void RPG_quad_ROS_subscriber<T>::TopicCallback(
        const std_msgs::Float32MultiArray::ConstPtr& my_array) const  {

 std::lock_guard<std::mutex> lock(key_);
 std::vector<float> temp = my_array->data;
 for (int i = 0; i < data_dimension_; ++i) {
    received_message_(i) = temp.at(i);
 }
  // received_message_ = Eigen::Map<Eigen::VectorXd> (temp.data(), data_dimension_);

}

template<typename T>
void RPG_quad_ROS_subscriber<T>::EvalOutput(const drake::systems::Context <T> &context,
                                           drake::systems::SystemOutput <T> *output) const {
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));
    std::lock_guard<std::mutex> lock(key_);
  drake::systems::System<T>::GetMutableOutputVector(output, 0) = received_message_;
 }


template<typename T>
void RPG_quad_ROS_subscriber<T>::DoPublish(
        const drake::systems::Context<T>& context) const {
 }

template class RPG_quad_ROS_subscriber<double>;

}   // namespace message_sys
}   // namespace ros