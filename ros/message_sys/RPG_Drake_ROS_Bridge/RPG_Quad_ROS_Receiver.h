#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A source bock which receives ROS messages

template <typename T>
class RPG_quad_ROS_receiver :public LeafSystem<T>{
 public:
    /// Constructs a system with a publisher to ROS

    RPG_quad_ROS_receiver();
    ~RPG_quad_ROS_receiver() override;

    void EvalOutput(const Context<T>& context,
                SystemOutput<T>* output) const override;

    // returns input port
    const SystemPortDescriptor<T>&get_input_port() const;

    /// Returns the output port
    const SystemPortDescriptor<T>&get_output_port() const;
};


}   // namespace systems
}   // namespace drake
