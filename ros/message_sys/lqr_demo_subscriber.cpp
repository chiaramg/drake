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
     sub_to_ros_ = nh_.subscribe(
             node_name, 100, &RPG_quad_ROS_subscriber<T>::TopicCallback, this);

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
        const std_msgs::Float32MultiArray::ConstPtr& my_array)  {

  std::cout << "This is the listener inside Drake, receiving message chiara rocks!" << std::endl;

    std::vector<float> temp = my_array->data;

   key_.lock();
   Eigen::VectorXf received_message_ = Eigen::Map<Eigen::VectorXf> (temp.data(), data_dimension_);
   //   RPG_quad_ROS_subscriber<T>::key_.unlock();
  key_.unlock();

    std::cout<<received_message_<<std::endl;

// std::cout << received_message_ << std::endl;
}

template<typename T>
void RPG_quad_ROS_subscriber<T>::EvalOutput(const drake::systems::Context <T> &context,
                                           drake::systems::SystemOutput <T> *output) const {
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  drake::systems::BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  //ros::spinOnce()
// std::cout << "In EvalOutput of the subscriber printing : " <<received_message_<< std::endl;

  // key_.lock();
///    std::vector<float> temp = received_message_;
    /*
    for(int i=0; i<12; i++){
     temp.push_back(received_message_(i));
    }*/
  //  key_.unlock();

    ///
/*    std::vector<float> temp;

    //temp.data.clear();

    for (int i = 0; i < 12; i++) {
        temp.push_back(received_message_(i));
    }
*/
///
    //Eigen::VectorXf test = Eigen::VectorXf::Ones(12);

    //key_.lock();
    //Eigen::VectorXf vec = received_message_;
  //  key_.unlock();

///    Eigen::VectorXf vec = Eigen::Map<Eigen::VectorXf> (temp.data(), 12);
    //RPG_quad_ROS_subscriber<T>::key_.lock();


    auto y = output_vector->get_mutable_value();
    key_.lock();
    y = received_message_;
    key_.unlock();


//  output_vector->get_mutable_value() = test;

  //RPG_quad_ROS_subscriber<T>::key_.unlock();

}


template<typename T>
void RPG_quad_ROS_subscriber<T>::DoPublish(
        const drake::systems::Context<T>& context) const {
            //DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
/*
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  std::cout << "In the DoPublish of the receiver" << std::endl;

  //RPG_quad_ROS_subscriber subscribe_to_ros;

  //ros::NodeHandle n;

  //std::cout << "NodeHandle created in the receiver" << std::endl;

  //ros::Subscriber sub = n.subscribe("chatter", 100, firstCallback);
  //messages_chiara ObstacleMessage(n);
  //std::cout << "obstacle of class messages_chiara created" << std::endl;

  //ros::Subscriber sub;

 // sub = n.subscribe("chatter1", 100, firstCallback, this);
  //sub = n.subscribe("chatter1", 100, firstCallback);


  //std::cout << "subscribed to Chatter1" << std::endl;

            //std::cout<<ObstacleMessage.received_message<<std::endl;
            // put received_message into outputport of the receiver!!
            //std::vector<float> temp = my_array->data;
            // Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);
            //Eigen::MatrixXf message_content = receiveMessage.received_message;
            //std_msgs::Float32MultiArray* my_array;
            //messages_chiara::firstCallback(&my_array);
            //System<T>::GetMutableOutputVector(output, 0) = ; //received message
*/
    ros::spinOnce();
}

template class RPG_quad_ROS_subscriber<double>;

}   // namespace message_sys
}   // namespace ros