//
// Created by chiara on 24.11.16.
//

#ifndef CATKIN_WS_EXMPL_CHIARA_H
#define CATKIN_WS_EXMPL_CHIARA_H

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"


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

#endif //CATKIN_WS_EXMPL_CHIARA_H

