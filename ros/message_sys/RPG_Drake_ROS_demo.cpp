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
#include "lqr_demo_publisher.h"

#include "ros/ros.h"

#include "quadrotor_plant.h"
#include "lqr_demo_publisher.h"
#include "lqr_demo_subscriber.h"


namespace ros {
namespace message_sys{
namespace {

int do_main(int argc, char *argv[]) {

  ros::init(argc, argv, "Drake_ROS_LQR_demo");

  ros::NodeHandle n;

  drake::lcm::DrakeLcm lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
          GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
          multibody::joints::kRollPitchYaw, tree.get());

  drake::systems::DiagramBuilder<double> builder;

  Eigen::VectorXd vec = Eigen::VectorXd::Ones(12);


    /// -------------------- ADDING THE SYSTEMS --------------------------///

  auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(vec);

  auto subscriber = builder.AddSystem<RPG_quad_ROS_subscriber<double>>(n, "chatter1", 12);

  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();

  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);

  auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));


  std::cout<<"Controller properties : A"<<controller->A()<<"\n";
  std::cout<<"Controller properties : B"<<controller->B()<<"\n";
  std::cout<<"Controller properties : C"<<controller->C()<<"\n";
  std::cout<<"Controller properties : D"<<controller->D()<<"\n";
  ////
  auto publisher = builder.AddSystem<lqr_demo_publisher<double>>(n, "chatter2", 4);


    /// -------------------- CONNECTING THE DIAGRAM --------------------------///

  builder.Connect(subscriber->get_output_port(), controller->get_input_port());

  //  builder.Connect(source->get_output_port(), controller->get_input_port());

  builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));

  builder.Connect(controller->get_output_port(), publisher->get_input_port());

  //builder.Connect(controller->get_output_port(), publisher->get_input_port(0));
  //builder.Connect(subscriber->get_output_port(), publisher->get_input_port(1));


    auto diagram = builder.Build();

    std::cout<<"diagram built"<<std::endl;

  systems::Simulator<double> simulator(*diagram);

  VectorX<double> x0 = VectorX<double>::Zero(12);

    auto diagram_context = diagram->CreateDefaultContext();
    systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
        simulator.get_mutable_context(), quadrotor);

    x0 = VectorX<double>::Random(12);

    simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetFromVector(x0);

      std::cout<<"about to initialize the simulator"<<std::endl;

    simulator.Initialize();

      std::cout<<"about to stepTo"<<std::endl;

    simulator.set_target_realtime_rate(1.0);
    simulator.StepTo(0.5);

  return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}