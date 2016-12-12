#include <cmath>
#include <iostream>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_input.h"

#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/linear_system.h"
#include "drake/systems/framework/primitives/zero_order_hold.h"



#include "RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.h"
#include "RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


//namespace drake{
//namespace systems{
namespace ros {
namespace message_sys {
namespace{

int do_main(int argc, char *argv[]) {

  //std::cout << "I entered message_sys do_main function" << std::endl;

  ros::init(argc, argv, "firstPublish");

  Eigen::VectorXd vec = Eigen::VectorXd::Ones(1);

  drake::systems::DiagramBuilder<double> builder;

  //std::cout << "I instantiated the builder" << std::endl;


  auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(vec);

  // create affine system as the source
  auto linear_system = builder.AddSystem<drake::systems::LinearSystem<double>>(
                    Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Identity(1,1),
                    Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,1));

  //std::cout << "I added an affine system to the builder" << std::endl;


  auto publisher = builder.AddSystem <RPG_quad_ROS_publisher < double >> ();

  //std::cout << "I added the publisher to the builder" << std::endl;

  //std::cout<<source.get_size()<<std::endl;

  //auto receiver = builder.AddSystem<RPG_quad_ROS_receiver<double>>();

  auto sink = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.1, 1);


  builder.Connect(source->get_output_port(), linear_system->get_input_port());
  builder.Connect(linear_system->get_output_port(), publisher->get_input_port());

  builder.Connect(publisher->get_output_port(), sink->get_input_port(0));

  //builder.ExportOutput(receiver->get_output_port());

  //std::cout << "I connected the diagram" << std::endl;

  auto diagram = builder.Build();
  //std::cout << "I built the diagram" << std::endl;

  drake::systems::Simulator<double> simulator(*diagram);


  diagram->SetDefaultState(simulator.get_mutable_context());

  simulator.Initialize();



  //std::cout << "I initialized the simulator" << std::endl;

  simulator.StepTo(10);

  //std::cout<<"I finished simulating "<< std::endl;

  return 0;
}


}
}// message_sys
} //ros


int main(int argc, char* argv[]){

    //std::cout<<"I entered message_sys main function"<<std::endl;

 return ros::message_sys::do_main(argc, argv);
    //std::cout<<"I'm the main function from the message_sys cpp file";
    //return 0;
}