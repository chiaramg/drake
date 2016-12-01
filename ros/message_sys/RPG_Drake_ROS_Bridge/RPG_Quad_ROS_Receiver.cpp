//
// Created by chiara on 30.11.16.
//
#include "msgs_chiara.h"

#include "drake/examples_chiara/message_sys/RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.h"

namespace drake {
namespace systems {

template <typename T>
RPG_quad_ROS_receiverer<T>::RPG_quad_ROS_publisher(

)


template <typename T>
const SystemPortDescriptor<T>&
RPG_quad_ROS_receiver<T>::get_output_port() const {
    return System<T>::get_output_port(0);
}

template <typename T>
void RPG_quad_ROS_receiver<T>::EvalOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
    DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));




}




}   // namespace systems
}   // namespace drake