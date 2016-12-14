#include "RPG_Quad_ROS_Publisher.h"


#include "drake/systems/framework/leaf_context.h"
#include <msgs_chiara.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_throw.h"


namespace ros {
namespace message_sys {

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

template<typename T>
RPG_quad_ROS_publisher<T>::RPG_quad_ROS_publisher() {
  this->DeclareInputPort(drake::systems::kVectorValued, 1);
  this->DeclareOutputPort(drake::systems::kVectorValued, 1);
}

template<typename T>
RPG_quad_ROS_publisher<T>::~RPG_quad_ROS_publisher() {}


template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_publisher<T>::get_input_port() const {
  return drake::systems::System<T>::get_input_port(0);
}


template<typename T>
const drake::systems::SystemPortDescriptor <T> &
RPG_quad_ROS_publisher<T>::get_output_port() const {
  return drake::systems::System<T>::get_output_port(0);
}
/*

        messages_chiara::messages_publisher(ros::NodeHandle& nh_)

        {
            //SUBSCRIBERS not required
            sub = nh_.subscribe("chat2", 100, &messages_publisher::firstCallback,this);

            //PUBLISHERS
            pub = nh_.advertise<std_msgs::Float32MultiArray>("chatter", 1);

        }

        messages_chiara::~messages_chiara(){}

        void messages_chiara::firstCallback(const std_msgs::Float32MultiArray::ConstPtr& my_array)
        {
            std::cout << "This is the initial talker, Message received" << std::endl;

            std::vector<float> temp = my_array->data;

            Eigen::MatrixXf my_mat = Eigen::Map<Eigen::MatrixXf> (temp.data(), 6, 5);

            std::cout << my_mat << std::endl;

        }





*/

template<typename T>
void RPG_quad_ROS_publisher<T>::EvalOutput(const drake::systems::Context <T> &context,
                                                   drake::systems::SystemOutput <T> *output) const {
  std::cout << "In the EvalOutput function of the publisher" << std::endl;

  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(drake::systems::System<T>::CheckValidContext(context));

  std::cout << "in EvalOutput, entering EvalVectorInput" << std::endl;

  const drake::systems::BasicVector<T>* input = this->EvalVectorInput(context, 0);

  std::cout << "in EvalOutput, exit EvalVectorInput" << std::endl;

  const auto& input_values = input->get_value();

  std_msgs::Float32MultiArray message_to_send;

  message_to_send.data.clear();

  int numbers_input = input_values.cols();

  for (int i =0; i<numbers_input; i++){
     message_to_send.data.push_back(input_values(i));
  }

  std::cout<<input_values<<std::endl;

  //ros::init(argc, argv, "publishVector");
  ros::NodeHandle nh;
  //messages_chiara firstMessage(n); // firstMessage.sub, firstMessage.pub, firstMessage.firstCallback
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("chatter2", 100);

  ros::Rate loop_rate(10);




  while (ros::ok()) {
     //firstMessage.pub.publish(message_to_send);
      chatter_pub.publish(message_to_send);
  }

/*
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
                std::cout << "message coming through...." << std::endl;
                ros::spinOnce();
                loop_rate.sleep();
            }

            //get_mutable_output(output);

*/
        }

template class RPG_quad_ROS_publisher<double>;

}   // namespace message_sys
}   // namespace ros
