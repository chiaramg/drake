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
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "lqr_demo_publisher.h"

#include "ros/ros.h"

#include "quadrotor_plant.h"

using namespace drake;

namespace ros {
namespace message_sys{
namespace {
/*
    DEFINE_double(realtime_factor,
    1.0,
    " "
    " ");
*/
int do_main(int argc, char *argv[]) {

        lcm::DrakeLcm lcm;

        RigidBodyTree<double> tree(
                GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
                multibody::joints::kRollPitchYaw);

       // gflags::ParseCommandLineFlags(&argc, &argv, true);

        systems::DiagramBuilder<double> builder;
        auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();

        std::cout<<"Qadrotor plant added"<< std::endl;

        //auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));
        auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));

        std::cout<<"LQR added"<< std::endl;

        auto publisher = builder.AddSystem<lqr_demo_publisher<double>>();

        /// ADD DRAKEVISUALIZER 
        auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);


        std::cout<<"publisher added"<< std::endl;

        builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
        builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));
        builder.Connect(controller->get_output_port(), publisher->get_input_port());

        /// CONNECT DRAKEVISUALIZER TO QUADROTOR OUTPUT
        builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

        std::cout<<"Diagram connected"<<std::endl;

        auto diagram = builder.Build();

        std::cout<<"Diagram built"<<std::endl;

        systems::Simulator<double> simulator(*diagram);
        systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
                simulator.get_mutable_context(), quadrotor);

        // set initial condition
        //auto context = quadrotor->CreateDefaultContext();

        //Eigen::VectorXf initial_state =
        //          quadrotor_context->get_mutable_continuous_state_vector().CopyToVector();
        //initial_state(2)=2.0;

        VectorX<double> x0 = VectorX<double>::Zero(12);
        x0(1) = 0.9;
        x0(2) = 1.0;
        x0(3) = 1.0;

        auto initial_context = quadrotor->CreateDefaultContext();
        quadrotor->set_state(initial_context.get(), x0);


        //simulator.set_target_realtime_rate(FLAGS_realtime_factor);

        simulator.Initialize();

        simulator.StepTo(10);

        return 0;
}

}
}  // namespace message_sys
}  // namespace ros

int main(int argc, char* argv[]) {
    return ros::message_sys::do_main(argc, argv);
}
