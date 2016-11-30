//
// Created by chiara on 30.11.16.
//

#pragma once

#ifndef CATKIN_WS_RPG_QUAD_ROS_PUBLISHER_H
#define CATKIN_WS_RPG_QUAD_ROS_PUBLISHER_H

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A sink block with an output port, producing ROS messages



template <typename T>

class RPG_quad_ROS_publisher : public LeafSystem<T>{
 public:
    /// Constructs a system with a publisher to ROS

    void EvalOutput(const )

    /// Returns the output port
    const SystemPortDescriptor<T>& get_output_port() const;
};


}   // namespace systems
}   // namespace drake




#endif //CATKIN_WS_RPG_QUAD_ROS_PUBLISHER_H
