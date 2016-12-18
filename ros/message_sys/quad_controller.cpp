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
/*
  drake::lcm::DrakeLcm lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
  GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
  multibody::joints::kRollPitchYaw, tree.get());
*/

  drake::systems::DiagramBuilder<double> builder;

  Eigen::VectorXd vec = Eigen::VectorXd::Ones(12);


  /// -------------------- ADDING THE SYSTEMS --------------------------///

  auto subscriber = builder.AddSystem<RPG_quad_ROS_subscriber<double>>(n, "controller_in", 12);

//  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);

  /// need quadrotos system, because the stabiling lqr depend on it...
  /// results in failure at initializihng, because of nullptr at input...
  ///-> connect to controller anyway? or create quad system with no in and output?

  std::unique_ptr<QuadrotorPlant<double>> quadrotor_ptr = std::make_unique<QuadrotorPlant<double>>();

    //std::unique_ptr<QuadrotorPlant> quadrotor_ptr = std::make_unique();

    //auto quadrotor_ptr = std::make_unique<QuadrotorPlant>();


  auto controller = builder.AddSystem(StabilizingLQRController(quadrotor_ptr.get()));

  auto zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.1, 4);

  ////
  auto publisher = builder.AddSystem<lqr_demo_publisher<double>>(n, "controller_out", 4);


  /// -------------------- CONNECTING THE DIAGRAM --------------------------///

  builder.Connect(subscriber->get_output_port(), controller->get_input_port());


//  builder.Connect(subscriber->get_output_port(), visualizer->get_input_port(0));


  builder.Connect(controller->get_output_port(), publisher->get_input_port());


  builder.Connect(controller->get_output_port(), zoh->get_input_port(0));



  //builder.Connect(controller->get_output_port(), publisher->get_input_port(0));
                //builder.Connect(subscriber->get_output_port(), publisher->get_input_port(1));

  auto diagram = builder.Build();


  systems::Simulator<double> simulator(*diagram);




/// new stuff added for visualization::
/*
    VectorX<double> x0 = VectorX<double>::Zero(12);


    for(int i = 0; i<10; i++) {

        auto diagram_context = diagram->CreateDefaultContext();

        std::cout<<"diagram_default context created"<<std::endl;


        systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
                simulator.get_mutable_context(), quadrotor_ptr.get());

        std::cout<<"is this sö problem"<<std::endl;


        x0 = VectorX<double>::Random(12);
        simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetFromVector(x0);

        std::cout<<"about to initialize"<<std::endl;


        simulator.Initialize();
        simulator.StepTo(4);
        simulator.reset_context(std::move(diagram_context));
    }
*/
/// end new stuff visualization


/*
  VectorX<double> x0 = VectorX<double>::Zero(12);

  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
            simulator.get_mutable_context(), quadrotor);

  x0 = VectorX<double>::Random(12);

  simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetFromVector(x0);

  std::cout<<"about to initialize the simulator"<<std::endl;
*/


/// stuff removed from visualization
    /// in case of emergeny, add again:

  std::cout<<"about to initialize"<<std::endl;

  simulator.Initialize();

  std::cout<<"about to stepTo"<<std::endl;

  simulator.StepTo(20.0);

///***********************************************************
  return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}