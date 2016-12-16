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

RPG_quad_ROS_subscriber(ros::NodeHandle& nh_);

~RPG_quad_ROS_subscriber() override;

void firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array);

void DoPublish(const drake::systems::Context<T>& context) const override;

void EvalOutput(const drake::systems::Context <T> &context,
                drake::systems::SystemOutput <T> *output) const override;

// returns input port
const drake::systems::SystemPortDescriptor <T> &get_input_port() const;

/// Returns the output port
const drake::systems::SystemPortDescriptor <T> &get_output_port() const;

private:
     Eigen::VectorXf received_message_;
     std::mutex key_;
     ros::Subscriber sub_to_ros_;

};

}   // namespace message_sys
}   // namespace ros
