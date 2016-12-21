#include "quadrotor_plant.h"
#include <math.h>

namespace ros {
namespace message_sys {

template <typename T>
class RPGQuadrotorPlant : public QuadrotorPlant<T> {
public:
  RPGQuadrotorPlant()
     : RPGQuadrotorPlant(0.5, 0.175, ((Eigen::Matrix3d() << 0.00371, 0, 0, 0,
                                                 0.00371, 0, 0, 0, 0.0040)
                                                 .finished()),
                                         1.0, 0.0245) {}

  ~RPGQuadrotorPlant() override;

  void EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const override;
};

}  // namespace message_sys
} // namespace ros