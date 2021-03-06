#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : source_value_(source_value) {
  const int n = static_cast<int>(source_value.rows());
  this->DeclareOutputPort(kVectorValued, n);
}

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(const T& source_value)
    : source_value_(Vector1<T>::Constant(source_value)) {
  this->DeclareOutputPort(kVectorValued, 1);
}

template <typename T>
const SystemPortDescriptor<T>& ConstantVectorSource<T>::get_output_port()
    const {
  return System<T>::get_output_port(0);
}

template <typename T>
void ConstantVectorSource<T>::EvalOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  System<T>::GetMutableOutputVector(output, 0) = source_value_;
}

// Explicitly instantiates on the most common scalar types.
template class ConstantVectorSource<double>;
template class ConstantVectorSource<AutoDiffXd>;
template class ConstantVectorSource<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
