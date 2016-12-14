#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_input.h"

#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.h"
#include "RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.h"



#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


namespace drake{
namespace systems{
//namespace ros {
//namespace message_sys {
//namespace RPG_Drake_Ros_Bridge{
//namespace{

        int do_main(int argc, char *argv[]) {

            std::cout << "I entered message_sys do_main function" << std::endl;

            ros::init(argc, argv, "firstPublish");

            Eigen::VectorXd vec = Eigen::VectorXd::Zero(1);

            systems::DiagramBuilder<double> builder;

            std::cout << "I instantiated the builder" << std::endl;


            //float send(2.3);

            auto source = builder.AddSystem<systems::ConstantVectorSource>(vec);
            //auto source = builder.AddSystem<systems::ConstantVectorSource>(send);
            std::cout << "I added ConstVectorSource to the builder" << std::endl;


            auto publisher = builder.AddSystem <RPG_quad_ROS_publisher < double >> ();

            std::cout << "I added the publisher to the builder" << std::endl;

            //auto publisher = builder.AddSystem(RPG_quad_ROS_publisher);
            //auto publisher = builder.AddSystem<RPG_quad_ROS_publisher>();

            //std::cout<<source.get_size()<<std::endl;

            auto receiver = builder.AddSystem<RPG_quad_ROS_receiver>();


            builder.Connect(source->get_output_port(), publisher->get_input_port());


            builder.Connect(publisher->get_output_port(), receiver->get_input_port());


            std::cout << "I connected the diagram" << std::endl;

            auto diagram = builder.Build();


            std::cout << "I built the diagram" << std::endl;

            systems::Simulator<double> simulator(*diagram);



            // systems::Context<double>* sim_context = 
            //   diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),

            simulator.Initialize();

            std::cout << "I initialized the simulator" << std::endl;

            simulator.StepTo(10);

            return 0;

        }


} // systems
//}
} //drake

#include <iostream>

int main(int argc, char* argv[]){

    std::cout<<"I entered message_sys main function"<<std::endl;

 return drake::systems::do_main(argc, argv);
    //std::cout<<"I'm the main function from the message_sys cpp file";
    //return 0;
}