#pragma once

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

namespace ros {
namespace message_sys {


        class messages_chiara {
        public:
            messages_chiara(ros::NodeHandle &nh_);

            ~messages_chiara();

            //Publishers
            ros::Publisher pub;

        private:
            //Subscribers
            ros::Subscriber sub;

            //Callbacks
            void firstCallback(const std_msgs::Float32MultiArray::ConstPtr &my_array);

        };


} // namespace message_sys
} // namespace ros