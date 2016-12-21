#include "RPG_Quadrotor_Plant.h"

namespace ros {
namespace message_sys {


template <typename T>
void RPGQuadrotorPlant<T>::EvalTimeDerivatives(
                    const systems::Context<T>& context,
                    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();

  Vector3<T> rpy = state.segment(3, 3);
  Vector3<T> rpy_dot = state.segment(9, 3);
  Matrix3<T> R = drake::math::rpy2rotmat(rpy);

  VectorX<T> uF = kf_ * u; // thrust
  VectorX<T> uM = km_ * u;   //

  Vector3<T> Fg(0, 0, -m_ * g_); // gravity
  Vector3<T> F(0, 0, uF.sum()); // total thrust
  Vector3<T> M(L_ / sqrt(2) * (uF(1) + uF(2) - uF(3) - uF(4)),
                             L_ / sqrt(2) * (uF(1) - uF(2) + uF(3) - uf(4)),
                             uM(0) - uM(1) + uM(2) - uM(3)); // torques
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

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
            const QuadrotorPlant<double>* quad) {

  auto quad_context_goal = quad->CreateDefaultContext();

  // --> steady state hover input
  quad_context_goal->FixInputPort(0, Eigen::VectorXd::Zero(4));

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0(0) = 0.0;
  x0(1) = 0.0;
  x0(2) = 1.0;

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(4);

  u0[4] =  quad->m() * quad->g()/4;

  quad_context_goal->FixInputPort(0, u0);
  quad->set_state(quad_context_goal.get(), x0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
  Q *= 10;

  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();

  return drake::systems::LinearQuadraticRegulator(*quad, *quad_context_goal, Q, R);
}


}  // namespace message_sys
} // namespace ros