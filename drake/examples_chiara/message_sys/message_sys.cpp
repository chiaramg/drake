//
// Created by chiara on 29.11.16.
//



#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


namespace drake{
namespace examples{
namespace RPG_Drake_Ros_Bridge{
namespace{

int do_main(int argc, char*argv[]){

    Eigen::VectorXd vec = Eigen::VectorXd::Zero(6);

    systems::DiagramBuilder builder;

    auto source = builder.AddSystem<system::ConstantVectorSource>(vec);

    auto publisher = builder.AddSystem<>();

    builder.Connect(source->get_output_port(0). publisher->get_input_port(0));

    auto diagram = builder.Build();

/********************  Publish to ROS: *************************/
/** Put the following somehow inside publisher output-port...  */
/***************************************************************/


    ros::init(argc, argv, "publishVector");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 100);

    ros::Rate loop_rate(10);

    // create eigen::matrix
/*    Eigen::MatrixXf eig_matrix(6,5);

    for(int i=0; i<6; i++){
        for(int j=0; j<5; j++){
            eig_matrix(i, j) = i*5+j;
        }
    }
*/
    std_msgs::Float32MultiArray my_vec;

    my_vec.data.clear();

    for (int i = 0; i < 6; i++) {
            my_vec.data.push_back(vec(i));
    }

    while(ros::ok()) {
        chatter_pub.publish(my_vec);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




}
} //message_sys
} //examples
} //drake



int main(int argc, char* argv[]){
 return drake::examples::message_sys::do_main(argc, argv);
}