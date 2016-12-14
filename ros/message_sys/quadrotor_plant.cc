#include "quadrotor_plant.h"

namespace drake {
namespace examples {
namespace quadrotor {

template <typename T>
QuadrotorPlant<T>::QuadrotorPlant(const double m, const double L,
                                  const Matrix3<T> I, const double kf,
                                  const double km)
    : g_(9.81), m_(m), L_(L), I_(I), kf_(kf), km_(km) {
  this->DeclareInputPort(systems::kVectorValued, 4);
  this->DeclareOutputPort(systems::kVectorValued, 12);
}

template <typename T>
QuadrotorPlant<T>::~QuadrotorPlant() {}

template <typename T>
QuadrotorPlant<AutoDiffXd>* QuadrotorPlant<T>::DoToAutoDiffXd() const {
  return new QuadrotorPlant<AutoDiffXd>(m_, L_, I_, kf_, km_);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
QuadrotorPlant<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  DRAKE_THROW_UNLESS(descriptor.get_size() == 12);
  return std::make_unique<systems::BasicVector<T>>(12);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
QuadrotorPlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<systems::BasicVector<T>>(12), 6 /* num_q */,
      6 /* num_v */, 0 /* num_z */);
}

template <typename T>
void QuadrotorPlant<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)
      ->set_value(context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void QuadrotorPlant<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  VectorX<T> uF = kf_ * u;
  VectorX<T> uM = km_ * u;

  Vector3<T> Fg(0, 0, -m_ * g_);
  Vector3<T> F(0, 0, uF.sum());
  Vector3<T> M(L_ * (uF(1) - uF(3)), L_ * (uF(2) - uF(0)),
               uM(0) - uM(1) + uM(2) - uM(3));
  Vector3<T> xyz_ddot = (1.0 / m_) * (Fg + R * F);

  Vector3<T> pqr;
  rpydot2angularvel(rpy, rpy_dot, pqr);
  pqr = R.adjoint() * pqr;

  Vector3<T> pqr_dot = I_.ldlt().solve(M - pqr.cross(I_ * pqr));
  Matrix3<T> Phi;
  typename drake::math::Gradient<Matrix3<T>, 3>::type dPhi;
  typename drake::math::Gradient<Matrix3<T>, 3, 2>::type* ddPhi = nullptr;
  angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

  MatrixX<T> drpy2drotmat = drake::math::drpy2rotmat(rpy);
  VectorX<T> Rdot_vec(9);
  Rdot_vec = drpy2drotmat * rpy_dot;
  Matrix3<T> Rdot = Eigen::Map<Matrix3<T>>(Rdot_vec.data());
  VectorX<T> dPhi_x_rpydot_vec;
  dPhi_x_rpydot_vec = dPhi * rpy_dot;
  Matrix3<T> dPhi_x_rpydot = Eigen::Map<Matrix3<T>>(dPhi_x_rpydot_vec.data());
  Vector3<T> rpy_ddot =
      Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

  VectorX<T> xdot(12);
  xdot << state.tail(6), xyz_ddot, rpy_ddot;
  derivatives->SetFromVector(xdot);
}

template class QuadrotorPlant<double>;
template class QuadrotorPlant<AutoDiffXd>;

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
