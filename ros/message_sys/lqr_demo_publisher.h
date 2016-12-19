#pragma once

#include <cstdint>
#include <memory>

//#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

namespace ros {
namespace message_sys {
/// A sink block with an output port, producing ROS messages

template<typename T>
class lqr_demo_publisher : public drake::systems::LeafSystem<T> {
public:
  /// Constructs a system with a publisher to ROS

  //explicit RPG_quad_ROS_publisher();
  lqr_demo_publisher(const ros::NodeHandle& nh, const std::string& node_name, const int& data_dimension);

  ~lqr_demo_publisher() override;

  //const int portIndex;

  void DoPublish(const drake::systems::Context<T>& context) const override;


  void EvalOutput(const drake::systems::Context <T> &context,
                    drake::systems::SystemOutput <T> *output) const override;

  // Returns the input port
  const drake::systems::SystemPortDescriptor <T> &get_input_port() const;
    const drake::systems::SystemPortDescriptor <T> &get_output_port() const;


private:
    ros::Publisher send_to_ros_;
    ros::NodeHandle nh_;
    int data_dimension_;
};

}   // namespace message_sys
}   // namespace ros