#include "RPG_Quad_ROS_Publisher.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>


namespace drake {
namespace systems{

//template <typename T>
//RPG_quad_ROS_publisher<T>::RPG_quad_ROS_publisher();
/*
messages_chiara(ros::NodeHandle& nh_)
{
    //SUBSCRIBERS
    sub = nh_.subscribe("chat2", 100, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    pub = nh_.advertise<std_msgs::Float32MultiArray>("chatter", 1);
}
*/


template <typename T>
const SystemPortDescriptor<T>&
RPG_quad_ROS_publisher<T>::get_output_port() const {
    return System<T>::get_output_port(0);
}

template <typename T>
void RPG_quad_ROS_publisher<T>::EvalOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
    DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

    //ros::init(argc, argv, "publishVector");
    ros::NodeHandle n;
    messages_chiara firstMessage(n); // firstMessage.sub, firstMessage.pub, firstMessage.firstCallback
    ros::Rate loop_rate(10);

    // create eigen::matrix

    Eigen::MatrixXf eig_matrix(6, 5);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            eig_matrix(i, j) = i * 5 + j;
        }
    }

    std_msgs::Float32MultiArray my_array;

    my_array.data.clear();

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            my_array.data.push_back(eig_matrix(i, j));
        }
    }


    while (ros::ok()) {

        firstMessage.pub.publish(my_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


}   // namespace systems
}   // namespace drake


// does not build without the main function...
// also when running, it's the following main function that gets called
// (not the one from the message_sys.cpp)

/*
#include <iostream>

int main(int argc, char* argv[]) {
    //return drake::systems::do_main(argc, argv);
    std::cout << "booah";
    return 0;
}
*/