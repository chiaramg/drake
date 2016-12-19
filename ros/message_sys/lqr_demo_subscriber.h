#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include "std_msgs/Float32MultiArray.h"
#include <ros/ros.h>

#include <mutex>

namespace ros {
namespace message_sys {
/// A source bock which receives ROS messages

 template<typename T>
class RPG_quad_ROS_subscriber : public drake::systems::LeafSystem<T> {
public:
/// Constructs a system with a publisher to ROS

RPG_quad_ROS_subscriber(const ros::NodeHandle& nh, const std::string& node_name, const int& data_dimension);
~RPG_quad_ROS_subscriber() override;


void TopicCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array) const ;

void DoPublish(const drake::systems::Context<T>& context) const override;

void EvalOutput(const drake::systems::Context <T> &context,
                drake::systems::SystemOutput <T> *output) const override;

/// Returns the output port
const drake::systems::SystemPortDescriptor <T> &get_output_port() const;

private:

     //ros::NodeHandle nh_;

     mutable Eigen::VectorXd received_message_;
     mutable std::mutex key_;
     ros::NodeHandle nh_;
     ros::Subscriber sub_to_ros_;
     ros::AsyncSpinner spinner{2};
     const int data_dimension_;

};

}   // namespace message_sys
}   // namespace ros
