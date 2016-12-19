#include <memory>

#include <gflags/gflags.h>

#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "ros/ros.h"

#include "quadrotor_plant.h"
#include "lqr_demo_publisher.h"
#include "lqr_demo_subscriber.h"


namespace ros {
namespace message_sys{
namespace {

int do_main(int argc, char *argv[]) {

  ros::init(argc, argv, "Quad_Simulator");

  ros::NodeHandle n;

  /// ADD FOR VISUALIZATION:


  drake::lcm::DrakeLcm lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                 GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
                        multibody::joints::kRollPitchYaw, tree.get());

  drake::systems::DiagramBuilder<double> builder;

  // -------------------- ADDING THE SYSTEMS --------------------------///

  auto subscriber = builder.AddSystem<RPG_quad_ROS_subscriber<double>>(n, "controller_out", 4);
  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();
  auto publisher = builder.AddSystem<lqr_demo_publisher<double>>(n, "controller_in", 12);
  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);

 // -------------------- CONNECTING THE DIAGRAM --------------------------///

  builder.Connect(subscriber->get_output_port(), quadrotor->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), publisher->get_input_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

/// ADD FOR VISUALIZATION PURPOSES

  VectorX<double> x0 = VectorX<double>::Zero(12);
  for(int i = 0; i<10; i++){

    auto diagram_context = diagram->CreateDefaultContext();
    systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
            simulator.get_mutable_context(), quadrotor);
    x0 = VectorX<double>::Random(12);

    simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetFromVector(x0);

    simulator.Initialize();

    simulator.StepTo(4);
    simulator.reset_context(std::move(diagram_context));
  }
  return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}