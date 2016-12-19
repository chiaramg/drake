#include "lqr_demo_publisher.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_throw.h"



namespace ros {
namespace message_sys {

template<typename T>
lqr_demo_publisher<T>::lqr_demo_publisher(const ros::NodeHandle& nh,
                                          const std::string& node_name,
                                          const int& data_dimension)
                                : nh_(nh),
                                  data_dimension_(data_dimension){
  //this->DeclareInputPort(drake::systems::kVectorValued, 4);
  this->DeclareInputPort(drake::systems::kVectorValued, data_dimension_);
  this->DeclareOutputPort(drake::systems::kVectorValued, 1);

  send_to_ros_ = nh_.advertise<std_msgs::Float32MultiArray>(node_name, 100);

}

template<typename T>
lqr_demo_publisher<T>::~lqr_demo_publisher() {}

template<typename T>
const drake::systems::SystemPortDescriptor <T> &
   lqr_demo_publisher<T>::get_input_port() const {
  return drake::systems::System<T>::get_input_port(0);
}

    template<typename T>
    const drake::systems::SystemPortDescriptor <T> &
    lqr_demo_publisher<T>::get_output_port() const {
        return drake::systems::System<T>::get_output_port(0);
    }


template <typename T>
void lqr_demo_publisher<T>::EvalOutput(const drake::systems::Context <T> &context,
                                       drake::systems::SystemOutput <T> *output) const {

}

template<typename T>
void lqr_demo_publisher<T>::DoPublish(const drake::systems::Context<T>& context) const {

  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  const drake::systems::BasicVector<T>* input = this->EvalVectorInput(context, 0);

  //const auto& input_values = input->get_value();

  std_msgs::Float32MultiArray message_to_send;
  Eigen::VectorXd input_values = input->get_value();

  //message_to_send_.data.clear();
  //int numbers_input = input_values.rows();

  for (int i =0; i<data_dimension_; i++){
    message_to_send.data.push_back(input_values(i));
    //message_to_send_.data(i) = input_values(i);
  }

//  std::cout<<"I send: "<<input_values<<std::endl;

  send_to_ros_.publish(message_to_send);

  ros::spinOnce();
}


template class lqr_demo_publisher<double>;

}   // namespace message_sys
}   // namespace ros