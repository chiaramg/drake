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
/*
messages_chiara::messages_chiara(ros::NodeHandle& nh_){

sub = nh_.subscribe("chatter1", 1, &messages_chiara::firstCallback,this);

std::cout << "subscribed to Chatter1" << std::endl;
}

messages_chiara::~messages_chiara(){}*/

template<typename T>
RPG_quad_ROS_subscriber<T>::RPG_quad_ROS_subscriber(ros::Nodehandle nh_) {
            //this->DeclareInputPort(drake::systems::kVectorValued, 1);
            //this->DeclareOutputPort(drake::systems::kVectorValued, 1);
      this->DeclareInputPort(drake::systems::kVectorValued, 1);
      this->DeclareOutputPort(drake::systems::kVectorValued, 12);
      sub_to_ros_ = nh_.subscribe("chatter1", 100, &RPG_quad_ROS_subscriber::firstCallback, this);
}

template<typename T>
RPG_quad_ROS_subscriber<T>::~RPG_quad_ROS_subscriber() {}

template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_subscriber<T>::get_input_port() const {
  return drake::systems::System<T>::get_input_port(0);
}

template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_subscriber<T>::get_output_port() const {
  return drake::systems::System<T>::get_output_port(0);
}

//void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)//,
  //const drake::systems::Context<double> &context,
  //drake::systems::SystemOutput<double> *output)
template<typename T>
void RPG_quad_ROS_subscriber<T>::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array) {
      //void firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array) {

  std::cout << "This is the listener inside Drake, receiving message" << std::endl;

  std::vector<float> temp = my_array->data;

  //this->received_message = my_array->received_message;
  //Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 12, 1);

  //    RPG_quad_ROS_subscriber<T>::key_.lock();
  Eigen::VectorXf received_message_ = Eigen::Map<Eigen::VectorXf> (temp.data(), 12);
   //   RPG_quad_ROS_subscriber<T>::key_.unlock();


  //received_message = my_mat;
  //drake::systems::BasicVector<double>* output_vector = output->GetMutableVectorData(0);
  //auto y = output_vector->get_mutable_value();



  std::cout << received_message_ << std::endl;
}

template<typename T>
void RPG_quad_ROS_subscriber<T>::EvalOutput(const drake::systems::Context <T> &context,
                                           drake::systems::SystemOutput <T> *output) const {
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  drake::systems::BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  std::cout << "BasicVector<T> output_vector" << std::endl;

  //RPG_quad_ROS_subscriber<T>::key_.lock();
  output_vector->get_mutable_value() = received_message_;
  //RPG_quad_ROS_subscriber<T>::key_.unlock();

}

template<typename T>
void RPG_quad_ROS_subscriber<T>::DoPublish(const drake::systems::Context<T>& context) const {
            //DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  std::cout << "In the receiver" << std::endl;

  ros::NodeHandle n;

  std::cout << "NodeHandle created in the receiver" << std::endl;

  //ros::Subscriber sub = n.subscribe("chatter", 100, firstCallback);
  //messages_chiara ObstacleMessage(n);
  //std::cout << "obstacle of class messages_chiara created" << std::endl;

  ros::Subscriber sub;

 // sub = n.subscribe("chatter1", 100, firstCallback, this);
  sub = n.subscribe("chatter1", 100, firstCallback);


  std::cout << "subscribed to Chatter1" << std::endl;

            //std::cout<<ObstacleMessage.received_message<<std::endl;
            // put received_message into outputport of the receiver!!
            //std::vector<float> temp = my_array->data;
            // Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);
            //Eigen::MatrixXf message_content = receiveMessage.received_message;
            //std_msgs::Float32MultiArray* my_array;
            //messages_chiara::firstCallback(&my_array);
            //System<T>::GetMutableOutputVector(output, 0) = ; //received message

  ros::spin();
}

template class RPG_quad_ROS_subscriber<double>;

}   // namespace message_sys
}   // namespace ros