#pragma once

#include <cstdint>
#include <memory>

//#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"
#include <ros/ros.h>


namespace ros {
namespace message_sys {
/// A sink block with an output port, producing ROS messages

template<typename T>
class drake_to_ros_publisher : public drake::systems::LeafSystem<T> {
public:
/// Constructs a system with a publisher to ROS

  drake_to_ros_publisher(const ros::NodeHandle& nh,
                         const std::string& node_name,
                         const int& data_dimension);

  ~drake_to_ros_publisher() override;

            //const int portIndex;

  void DoPublish(const drake::systems::Context<T>& context) const override;


  void EvalOutput(const drake::systems::Context <T> &context,
                  drake::systems::SystemOutput <T> *output) const override;

  // Returns the input port
  const drake::systems::SystemPortDescriptor <T> &get_input_port() const;

private:
  ros::Publisher send_to_ros_;
  ros::NodeHandle nh_;
  int data_dimension_;

};

}   // namespace message_sys
}   // n