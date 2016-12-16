#pragma once

#include <iostream>

#include <Eigen/Core>

#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/affine_system.h"


using namespace drake;

namespace ros {
namespace message_sys {


template <typename T>
class QuadrotorPlant : public drake::systems::LeafSystem<T> {
 public:
  QuadrotorPlant()
      : QuadrotorPlant(0.5, 0.175, ((Eigen::Matrix3d() << 0.0023, 0, 0, 0,
                                     0.0023, 0, 0, 0, 0.0040)
                                        .finished()),
                       1.0, 0.0245) {}
  QuadrotorPlant(const double m, const double L, const drake::Matrix3<T> I,
                 const double kf, const double km);


    /// The input force to this system is not direct feedthrough.
    bool has_any_direct_feedthrough() const override { return false; }

  ~QuadrotorPlant() override;

  QuadrotorPlant<drake::AutoDiffXd>* DoToAutoDiffXd() const override;

  void EvalOutput(const drake::systems::Context<T>& context,
                  drake::systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const drake::systems::Context<T>& context,
      drake::systems::ContinuousState<T>* derivatives) const override;
  int get_input_size() { return 4; }

  int get_num_states() { return 12; }

  void set_state(drake::systems::Context<T>* context, const VectorX<T> x) const {
    context->get_mutable_continuous_state_vector()->SetFromVector(x);
  }

  /// CHIARA
/*
    void SetDefaultState(const drake::systems::Context<T>& context,
                         drake::systems::State<T>* state) const override {
        DRAKE_DEMAND(state != nullptr);
        drake::systems::Diagram<T>::SetDefaultState(context, state);
        drake::systems::State<T>* plant_state =
                this->GetMutableSubsystemState(state, plant_);
        VectorX<T> x0(plant_->get_num_states());
        x0.setZero();
        // x0 is the initial state where
         // x0(0), x0(1), x0(2) are the quadrotor's x, y, z -states
         // x0(3), x0(4), x0(5) are the quedrotor's Euler angles phi, theta, psi
         //
        x0(2) = 0.2;  // setting arbitrary z-position
        plant_->set_state_vector(plant_state, x0);
    }
*/

   const double m() const {
        return m_;
    }

   const double g() const {
        return g_;
    }

protected:
  std::unique_ptr<drake::systems::ContinuousState<T>> AllocateContinuousState()
      const override;

  std::unique_ptr<drake::systems::BasicVector<T>> AllocateOutputVector(
      const drake::systems::SystemPortDescriptor<T>& descriptor) const override;



 private:

  const double g_;
  const double m_;
  const double L_;
  const Matrix3<T> I_;
  const double kf_;
  const double km_;
};

    ///CHIARA
    std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
            const QuadrotorPlant<double>* acrobot);

}  // namespace message_sys
}  // namespace ros
