#include <memory>


#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake_to_ros_publisher.h"
#include "drake/systems/primitives/linear_system.h"

#include "ros/ros.h"

#include "quadrotor_plant.h"

#include "drake_to_ros_publisher.h"


namespace ros {
namespace message_sys{
namespace {

int do_main(int argc, char *argv[]) {

  ros::init(argc, argv, "Drake_ROS_LQR_demo");

  ros::NodeHandle n;

  drake::systems::DiagramBuilder<double> builder;

  Eigen::VectorXd vec = Eigen::VectorXd::Ones(4);

    for(int i = 0; i<4; i++){
        vec (i) = i*i;
    }

  /// -------------------- ADDING THE SYSTEMS --------------------------///

  auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(vec);
  auto linsys = builder.AddSystem<drake::systems::LinearSystem<double>>(
          Eigen::MatrixXd::Identity(4, 4), Eigen::MatrixXd::Identity(4, 4),
          Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Identity(4, 4)
  );

  auto zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.1, 1) ;

  auto publisher = builder.AddSystem<drake_to_ros_publisher<double>>(n, "chatter2", 4);

  /// -------------------- CONNECTING THE DIAGRAM --------------------------///

    builder.Connect(source->get_output_port(), linsys->get_input_port());
  std::cout<<"source attached to linsys\n";
    builder.Connect(linsys->get_output_port(), publisher->get_input_port());
  std::cout<<"linsys attached to publisher\n";
    builder.Connect(publisher->get_output_port(0), zoh->get_input_port(0));
  std::cout<<"publisher attached to zoh\n";
    //builder.Connect(source->get_output_port(), publisher->get_input_port());

  auto diagram = builder.Build();

  std::cout<<"diagram built"<<std::endl;

  systems::Simulator<double> simulator(*diagram);

  diagram->SetDefaults(simulator.get_mutable_context());
  std::cout<<"about to initialize the simulator"<<std::endl;

  simulator.Initialize();

  simulator.set_target_realtime_rate(1.0);
  std::cout<<"about to stepTo"<<std::endl;

simulator.StepTo(0.5);

return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}