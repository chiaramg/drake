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

// from Philips implementations:
//#include "quadrotor_plant.h"
//#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


//namespace drake{
//namespace systems{
namespace ros {
namespace message_sys {
namespace{

int do_main(int argc, char *argv[]) {

  ros::init(argc, argv, "firstPublish");

  drake::systems::DiagramBuilder<double> builder;


  ///  WORKING DIAGRAM LINEAR SYSTEM-PUBLISHER

  Eigen::VectorXd vec = Eigen::VectorXd::Ones(12);
  auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(vec);

  auto linear_system = builder.AddSystem<drake::systems::LinearSystem<double>>(
                    Eigen::MatrixXd::Identity(12,12), Eigen::MatrixXd::Identity(12,4),
                    Eigen::MatrixXd::Identity(4,12), Eigen::MatrixXd::Zero(4,4));

  auto publisher = builder.AddSystem <RPG_quad_ROS_publisher < double >> ();

  //auto sink = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.1, 1);

  builder.Connect(source->get_output_port(), linear_system->get_input_port());
  builder.Connect(linear_system->get_output_port(), publisher->get_input_port());
  //builder.Connect(publisher->get_output_port(), sink->get_input_port(0));


  ///  IN PROGRESS: SYSTEM WITH RECEIVER-TO-PUBLISHER
/*
  Eigen::VectorXd vec = Eigen::VectorXd::Ones(1);
  auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(vec);

  auto receiver = builder.AddSystem<RPG_quad_ROS_receiver<double>>();

  std::cout << "Added the receiver to the system" << std::endl;

  auto publisher = builder.AddSystem<RPG_quad_ROS_publisher<double>>();
  //auto sink = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(0.1, 1);

  builder.Connect(source->get_output_port(), receiver->get_input_port());
  builder.Connect(receiver->get_output_port(), publisher->get_input_port());
  //builder.Connect(publisher->get_output_port(), sink->get_input_port(0));

  std::cout << "whole diagram connected" << std::endl;

*/
  ///   IN PROGRESS: LQR
/*
  //MatrixX<double> Q = MatrixX<double>::Identity(12,12);
  Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(12, 12);
  //Q << Eigen::MatrixXf::Identity;
  //R *= 10.0;
  Q.block<3,3>(0,0) *= 10.0;
  Q.block<3,3>(9,9) *= 10.0;

  Eigen::MatrixXf R = Eigen::MatrixXf::Identity(4,4);
  //R *= 10.0;
  //Eigen::MatrixXf R(4, 4) = Eigen::MatrixXf::Identity(4,4);
  R *= 10.0;

  // we want to receive the state of the quadrotor:
  auto receiver = builder.AddSystem<RPG_quad_ROS_receiver<double>>();

//  auto quad = builder.AddSystem<QuadrotorPlant<double>>();
  // system with 4 inputs: thrusts for each rotor & 12 outputs (statevector)

//  auto quad_context_goal = quad->CreateDefaultContext();
  //quad->set_state(quad_context_goal.get(), xg);

//  auto controller = builder.AddSystem(drake::systems::LinearQuadraticRegulator(
                                                //*quad, *quad_context_goal, Q, R));

  auto publisher = builder.AddSystem<RPG_quad_ROS_publisher<double>>();

//  builder.Connect(receiver->get_output_port(0), controller->get_input_port());
//  builder.Connect(controller->get_output_port(),quad->get_input_port(0)); // u = -K*x
//  builder.Connect(controller->get_output_port(0), publisher->get_input_port(0)); // K
*/

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  diagram->SetDefaultState(simulator.get_mutable_context());

  simulator.Initialize();

  std::cout << "Simulator initialized" << std::endl;

  simulator.StepTo(10);  //goes into EvalOutput of the publisher

  std::cout << "Simulator StepTo" << std::endl;

  std::cout<<"I finished simulating "<< std::endl;

  return 0;
}

}
}// message_sys
} //ros

int main(int argc, char* argv[]){

    //std::cout<<"I entered message_sys main function"<<std::endl;

 return ros::message_sys::do_main(argc, argv);
    //std::cout<<"I'm the main function from the message_sys_demo.cc file";
    //return 0;
}