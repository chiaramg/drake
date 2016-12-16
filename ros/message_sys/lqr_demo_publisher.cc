#include "lqr_demo_publisher.h"

#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_throw.h"

namespace ros {
    namespace message_sys {

        template<typename T>
        lqr_demo_publisher<T>::lqr_demo_publisher() {
            this->DeclareInputPort(drake::systems::kVectorValued, 4);
            this->DeclareOutputPort(drake::systems::kVectorValued, 1);
        }

        template<typename T>
        lqr_demo_publisher<T>::~lqr_demo_publisher() {}

        template<typename T>
        const drake::systems::SystemPortDescriptor <T> &
        lqr_demo_publisher<T>::get_input_port() const {
            return drake::systems::System<T>::get_input_port(0);
        }

        template<typename T>
        const drake::systems::SystemPortDescriptor <T> &
        lqr_demo_publisher<T>::get_output_port() const {
            return drake::systems::System<T>::get_output_port(0);
        }

        template<typename T>
        void lqr_demo_publisher<T>::EvalOutput(const drake::systems::Context <T> &context,
                                                   drake::systems::SystemOutput <T> *output) const {
            std::cout << "In the EvalOutput function of the publisher" << std::endl;

        }

        template<typename T>
        void lqr_demo_publisher<T>::DoPublish(const drake::systems::Context<T>& context) const {

            std::cout << "In the doPublish function of the publisher" << std::endl;

            DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

            std::cout << "in doPublish, entering EvalVectorInput" << std::endl;

            const drake::systems::BasicVector<T>* input = this->EvalVectorInput(context, 0);

            std::cout << "in doPublish, exit EvalVectorInput" << std::endl;

            const auto& input_values = input->get_value();

            std_msgs::Float32MultiArray message_to_send;

            message_to_send.data.clear();

            int numbers_input = input_values.cols();

            for (int i =0; i<numbers_input; i++){
                message_to_send.data.push_back(input_values(i));
            }


            std::cout<<input_values<<std::endl;

            std::cout<<"about to start ROS node"<<std::endl;

            //ros::init(argc, argv, "publishVector");

            ros::NodeHandle nh;

            std::cout<<"ROS node started"<<std::endl;


            //messages_chiara firstMessage(n); // firstMessage.sub, firstMessage.pub, firstMessage.firstCallback
            ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("chatter2", 100);

            ros::Rate loop_rate(10);

            std::cout<<"publish a message"<<std::endl;


            while (ros::ok()) {
                //firstMessage.pub.publish(message_to_send);
                chatter_pub.publish(message_to_send);
            }
        }


        template class lqr_demo_publisher<double>;

    }   // namespace message_sys
}   // namespace ros
