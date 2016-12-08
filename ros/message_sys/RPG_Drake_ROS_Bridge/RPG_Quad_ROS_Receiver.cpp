#include "RPG_Quad_ROS_Receiver.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>


namespace ros {
namespace message_sys {

//template <typename T>
//RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver();

/*

messages_chiara(ros::NodeHandle& nh_) {

    //SUBSCRIBERS
    sub = nh_.subscribe("chatter", 1, &messages_chiara::firstCallback,this);

    //PUBLISHERS
    //pub = nh_.advertise<std_msgs::Float32MultiArray>("chat2", 100);
}

*/

        template<typename T>
        RPG_quad_ROS_receiver<T>::RPG_quad_ROS_receiver() {
            this->DeclareInputPort(drake::systems::kVectorValued, 1, drake::systems::kContinuousSampling);
            this->DeclareOutputPort(drake::systems::kVectorValued, 1, drake::systems::kContinuousSampling);
        }

        template<typename T>
        RPG_quad_ROS_receiver<T>::~RPG_quad_ROS_receiver() {}

        template<typename T>
        const drake::systems::SystemPortDescriptor <T> &
        RPG_quad_ROS_receiver<T>::get_input_port() const {
            return drake::systems::System<T>::get_input_port(0);
        }

        template<typename T>
        const drake::systems::SystemPortDescriptor <T> &
        RPG_quad_ROS_receiver<T>::get_output_port() const {
            return drake::systems::System<T>::get_output_port(0);
        }

        template<typename T>
        void RPG_quad_ROS_receiver<T>::EvalOutput(const drake::systems::Context <T> &context,
                                                  drake::systems::SystemOutput <T> *output) const {
            DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
            DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

            std::cout << "In the receiver" << std::endl;

            ros::NodeHandle n;
            messages_chiara receiveMessage(n);


            //ros::spin();
        }

        template
        class RPG_quad_ROS_receiver<double>;



}   // namespace message_sys
}   // namespace ros


