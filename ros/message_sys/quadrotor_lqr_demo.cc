#include <memory>

#include <gflags/gflags.h>

//#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
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
//#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "quadrotor_plant.h"

using namespace drake;

namespace ros {
namespace message_sys{
int do_main(int argc, char* argv[]){

/*RigidBodyTree<double> tree(
            GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
            multibody::joints::kRollPitchYaw);
*/
    auto tree = std::make_unique<RigidBodyTree<double>>();
        parsers::urdf::AddModelInstanceFromUrdfFile(
            GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
            multibody::joints::kRollPitchYaw, nullptr, tree.get());


    //parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    //        GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
    //        multibody::joints::kRollPitchYaw, tree.get());

  systems::DiagramBuilder<double> builder;
  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();

  //auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));
  auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));

  builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* quadrotor_context =
            diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                                quadrotor);

    // set initial condition
    //auto context = quadrotor->CreateDefaultContext();

    //Eigen::VectorXf initial_state =
      //          quadrotor_context->get_mutable_continuous_state_vector().CopyToVector();
    //initial_state(2)=2.0;

  simulator.Initialize();

  simulator.StepTo(10);

  return 0;

}


}  // namespace quadLQR
}  // namespace drake

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}
