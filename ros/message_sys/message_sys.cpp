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

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


namespace drake{
namespace systems{
//namespace RPG_Drake_Ros_Bridge{
//namespace{

int do_main(int argc, char*argv[]) {

    ros::init(argc, argv, "firstPublish");

    Eigen::VectorXd vec = Eigen::VectorXd::Zero(6);

    systems::DiagramBuilder<double> builder;

    auto source = builder.AddSystem<systems::ConstantVectorSource>(vec);

    auto publisher = builder.AddSystem<RPG_quad_ROS_publisher>();

    builder.Connect(source->get_output_port(), publisher->get_input_port(0));

    auto diagram = builder.Build();

/*
    systems::Simulator<double> simulator(*diagram);

    simulator.Initialize();
    simulator.StepTo(10);
*/
    return 0;

}
///********************  Publish to ROS: *************************/
///** Put the following somehow inside publisher output-port...  */
///***************************************************************/
//
//
//    ros::init(argc, argv, "publishVector");
//    ros::NodeHandle n;
//    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 100);
//
//    ros::Rate loop_rate(10);
//
//    // create eigen::matrix
///*    Eigen::MatrixXf eig_matrix(6,5);
//
//    for(int i=0; i<6; i++){
//        for(int j=0; j<5; j++){
//            eig_matrix(i, j) = i*5+j;
//        }
//    }
//*/
//    std_msgs::Float32MultiArray my_vec;
//
//    my_vec.data.clear();
//
//    for (int i = 0; i < 6; i++) {
//            my_vec.data.push_back(vec(i));
//    }
//
//    while(ros::ok()) {
//        chatter_pub.publish(my_vec);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//
//    return 0;
//}
//
//
//
//
//}
//} //message_sys



} // systems
} //drake


// builds also without a main function
#include <iostream>

int main(int argc, char* argv[]){
 //return drake::systems::do_main(argc, argv);
    std::cout<<"booah, eeey";
    return 0;
}