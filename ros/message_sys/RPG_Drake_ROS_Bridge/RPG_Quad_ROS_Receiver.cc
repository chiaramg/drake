#include "RPG_Quad_ROS_Receiver.h"

#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace ros {
namespace message_sys {

//template <typename T>
//RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver();


messages_chiara::messages_chiara(ros::NodeHandle& nh_){

    //SUBSCRIBERS

    sub = nh_.subscribe("chatter1", 1, &messages_chiara::firstCallback,this);

    std::cout << "subscribed to Chatter1" << std::endl;
    //PUBLISHERS
    //pub = nh_.advertise<std_msgs::Float32MultiArray>("chat2", 100);
}


messages_chiara::~messages_chiara(){}


template<typename T>
RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver() {
  this->DeclareInputPort(drake::systems::kVectorValued, 1);
  this->DeclareOutputPort(drake::systems::kVectorValued, 1);
}

template<typename T>
RPG_quad_ROS_receiver<T>::~RPG_quad_ROS_receiver() {}

template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_receiver<T>::get_input_port() const {
  return drake::systems::System<T>::get_input_port(0);
}

template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_receiver<T>::get_output_port() const {

  return drake::systems::System<T>::get_output_port(0);
}

/*
messages_chiara::messages_chiara(ros::NodeHandle& nh_) {
  //SUBSCRIBERS
  sub = nh_.subscribe("chatter", 1, &messages_chiara::firstCallback,this);

  //PUBLISHERS, not required
  pub = nh_.advertise<std_msgs::Float32MultiArray>("chat2", 100);
}*/


void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)//,
                                    //const drake::systems::Context<double> &context,
                                    //drake::systems::SystemOutput<double> *output)
{


  std::cout << "This is the listener inside Drake, receiving message" << std::endl;

  std::vector<float> temp = my_array->data;

  //this->received_message = my_array->received_message;


  Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);

  received_message = my_mat;

  //drake::systems::BasicVector<double>* output_vector = output->GetMutableVectorData(0);
  //auto y = output_vector->get_mutable_value();

  std::cout << my_mat << std::endl;

  //pub.publish(my_array);

  //std::cout << "sent the same matrix back" << std::endl;

}

template<typename T>
void RPG_quad_ROS_receiver<T>::EvalOutput(const drake::systems::Context <T> &context,
                                                  drake::systems::SystemOutput <T> *output) const {
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  drake::systems::BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  std::cout << "BasicVector<T> output_vector" << std::endl;

  auto y = output_vector->get_mutable_value();
  std::cout << "auto y" << std::endl;

/*  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  std::cout << "In the receiver" << std::endl;
    
  ros::NodeHandle n;

  std::cout << "NodeHandle created in the receiver" << std::endl;


  //ros::Subscriber sub = n.subscribe("chatter", 100, firstCallback);

  messages_chiara ObstacleMessage(n);

  std::cout << "obstacle of class messages_chiara created" << std::endl;


  //std::cout<<ObstacleMessage.received_message<<std::endl;

  // put received_message into outputport of the receiver!!


  //std::vector<float> temp = my_array->data;

  // Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);


  //Eigen::MatrixXf message_content = receiveMessage.received_message;

  drake::systems::BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  std::cout << "BasicVector<T> output_vector" << std::endl;

  auto y = output_vector->get_mutable_value();
  std::cout << "auto y" << std::endl;

  //std_msgs::Float32MultiArray* my_array;

  //messages_chiara::firstCallback(&my_array);
  //System<T>::GetMutableOutputVector(output, 0) = ; //received message
  //ros::spin();
*/
}


template<typename T>
void RPG_quad_ROS_receiver<T>::DoPublish(const drake::systems::Context<T>& context) const {
  //DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  std::cout << "In the receiver" << std::endl;

  ros::NodeHandle n;

  std::cout << "NodeHandle created in the receiver" << std::endl;


  //ros::Subscriber sub = n.subscribe("chatter", 100, firstCallback);

  messages_chiara ObstacleMessage(n);

  std::cout << "obstacle of class messages_chiara created" << std::endl;


  //std::cout<<ObstacleMessage.received_message<<std::endl;

  // put received_message into outputport of the receiver!!


  //std::vector<float> temp = my_array->data;

  // Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);


  //Eigen::MatrixXf message_content = receiveMessage.received_message;



  //std_msgs::Float32MultiArray* my_array;

  //messages_chiara::firstCallback(&my_array);
  //System<T>::GetMutableOutputVector(output, 0) = ; //received message
  //ros::spin();

}




template class RPG_quad_ROS_receiver<double>;

}   // namespace message_sys
}   // namespace ros


