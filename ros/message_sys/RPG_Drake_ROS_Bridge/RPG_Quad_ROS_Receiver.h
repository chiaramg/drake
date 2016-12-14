#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace ros {
namespace message_sys {
/// A source bock which receives ROS messages

template<typename T>
class RPG_quad_ROS_receiver : public drake::systems::LeafSystem<T> {
public:
  /// Constructs a system with a publisher to ROS

  RPG_quad_ROS_receiver();

  ~RPG_quad_ROS_receiver() override;

  void DoPublish(const drake::systems::Context<T>& context) const override;

  void EvalOutput(const drake::systems::Context <T> &context,
                            drake::systems::SystemOutput <T> *output) const override;

  // returns input port
  const drake::systems::SystemPortDescriptor <T> &get_input_port() const;

  /// Returns the output port
  const drake::systems::SystemPortDescriptor <T> &get_output_port() const;
};



}   // namespace message_sys
}   // namespace ros
