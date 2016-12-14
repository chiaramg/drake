#pragma once

#include <iostream>

#include <Eigen/Core>

#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace quadrotor {

template <typename T>
class QuadrotorPlant : public systems::LeafSystem<T> {
 public:
  QuadrotorPlant()
      : QuadrotorPlant(0.5, 0.175, ((Eigen::Matrix3d() << 0.0023, 0, 0, 0,
                                     0.0023, 0, 0, 0, 0.0040)
                                        .finished()),
                       1.0, 0.0245) {}
  QuadrotorPlant(const double m, const double L, const Matrix3<T> I,
                 const double kf, const double km);

  ~QuadrotorPlant() override;

  QuadrotorPlant<AutoDiffXd>* DoToAutoDiffXd() const override;

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;
  int get_input_size() { return 4; }

  int get_num_states() { return 12; }

  void set_state(systems::Context<T>* context, const VectorX<T> x) const {
    context->get_mutable_continuous_state_vector()->SetFromVector(x);
  }

 protected:
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  const double g_;
  const double m_;
  const double L_;
  const Matrix3<T> I_;
  const double kf_;
  const double km_;
};

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
