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
#include "drake/systems/primitives/zero_order_hold.h"



namespace ros {
namespace message_sys{
namespace {

int do_main(int argc, char *argv[]) {

  ros::init(argc, argv, "Quad_Controller");

  ros::NodeHandle n;

  drake::systems::DiagramBuilder<double> builder;

  // -------------------- ADDING THE SYSTEMS --------------------------//

  auto subscriber = builder.AddSystem<RPG_quad_ROS_subscriber<double>>(n, "controller_in", 12);

  std::unique_ptr<QuadrotorPlant<double>> quadrotor_ptr = std::make_unique<QuadrotorPlant<double>>();
  //  std::unique_ptr<RPGQuadrotorPlant<double>> quadrotor_ptr = std::make_unique<RPGQuadrotorPlant<double>>();


  auto controller = builder.AddSystem(StabilizingLQRController(quadrotor_ptr.get()));
  auto zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.01, 1);
  auto publisher = builder.AddSystem<lqr_demo_publisher<double>>(n, "controller_out", 4);


  /// -------------------- CONNECTING THE DIAGRAM --------------------------///

  builder.Connect(subscriber->get_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), publisher->get_input_port());
  builder.Connect(publisher->get_output_port(), zoh->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  simulator.Initialize();

  simulator.StepTo(1000.0);

///***********************************************************
  return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}