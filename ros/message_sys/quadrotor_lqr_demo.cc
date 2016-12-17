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
//#include "lqr_demo_publisher.h"

#include "ros/ros.h"

#include "quadrotor_plant.h"

namespace ros {
namespace message_sys{
namespace {

int do_main(int argc, char *argv[]) {

        ros::init(argc, argv, "LQR_demo");

        drake::lcm::DrakeLcm lcm;

        auto tree = std::make_unique<RigidBodyTree<double>>();
        parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
                multibody::joints::kRollPitchYaw, tree.get());

        drake::systems::DiagramBuilder<double> builder;
        auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();

        auto controller = builder.AddSystem(StabilizingLQRController(quadrotor));

 //       auto publisher = builder.AddSystem<lqr_demo_publisher<double>>();

        /// ADD DRAKEVISUALIZER 
        auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);

        builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
        builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));

//        builder.Connect(controller->get_output_port(), publisher->get_input_port());

        /// CONNECT DRAKEVISUALIZER TO QUADROTOR OUTPUT
        builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

        auto diagram = builder.Build();

        systems::Simulator<double> simulator(*diagram);

        VectorX<double> x0 = VectorX<double>::Zero(12);


        for(int i = 0; i<10; i++){

            auto diagram_context = diagram->CreateDefaultContext();
            systems::Context<double> *quadrotor_context = diagram->GetMutableSubsystemContext(
                    simulator.get_mutable_context(), quadrotor);
            x0 = VectorX<double>::Random(12);

            //std::cout<<"x0:"<<std::endl;
            //std::cout<<x0<<std::endl;

            //quadrotor->set_state(initial_context.get(), x0);

            //std::cout<<"initial simulator context"<<std::endl;
            //std::cout<<simulator.get_mutable_context()->get_mutable_continuous_state_vector()->CopyToVector()<<std::endl;

            //initial_context->get_mutable_continuous_state_vector()->SetFromVector(x0);
            simulator.get_mutable_context()->get_mutable_continuous_state_vector()->SetFromVector(x0);

            //std::cout<<"initial simulator context after setting it to x0"<<std::endl;
            //std::cout<<simulator.get_mutable_context()->get_mutable_continuous_state_vector()->CopyToVector()<<std::endl;

            simulator.Initialize();

            //auto context_ = simulator.get_mutable_context();
            //std::cout<<"simulator StepTo:"<<std::endl;
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
