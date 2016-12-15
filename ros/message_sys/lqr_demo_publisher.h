#pragma once

#include <cstdint>
#include <memory>

//#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"


namespace ros {
    namespace message_sys {
/// A sink block with an output port, producing ROS messages

        template<typename T>
        class lqr_demo_publisher : public drake::systems::LeafSystem<T> {
        public:
            /// Constructs a system with a publisher to ROS

            //explicit RPG_quad_ROS_publisher();
            lqr_demo_publisher();

            ~lqr_demo_publisher() override;

            void DoPublish(const drake::systems::Context<T>& context) const override;

            void EvalOutput(const drake::systems::Context <T> &context,
                            drake::systems::SystemOutput <T> *output) const override;

            // Returns the input port
            const drake::systems::SystemPortDescriptor <T> &get_input_port() const;

            /// Returns the output port
            const drake::systems::SystemPortDescriptor <T> &get_output_port() const;
        };


    }   // namespace message_sys
}   // namespace ros

