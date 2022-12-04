#pragma once

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Bound constraint
 */
class BoundConstraint {
 public:
  BoundConstraint() {}

  BoundConstraint* clone() const {
    return new BoundConstraint(*this);
  }

  // True if no bound constraints are active on either state or input
  bool empty() const {
    return state_idx_.size() == 0 && input_idx_.size() == 0;
  }

  void setZero(size_t state_dim, size_t input_dim) {
    state_lb_.setZero(state_dim);
    state_ub_.setZero(state_dim);
    input_lb_.setZero(input_dim);
    input_ub_.setZero(input_dim);
  }

  // Get the constraint violation: a negative value has violated the constraint
  vector_t violation(const vector_t& state, const vector_t& input) const {
    vector_t v(2 * state.size() + 2 * input.size());
    v << state_ub_ - state, state - state_lb_, input_ub_ - input, input - input_lb_;
    return v;
  }

  // Center the constraints around a given state and input
  BoundConstraint center(const vector_t& state, const vector_t& input) const {
        BoundConstraint centered;
        centered.state_lb_ = state_lb_ - state;
        centered.state_ub_ = state_ub_ - state;
        centered.input_lb_ = input_lb_ - input;
        centered.input_ub_ = input_ub_ - input;
        centered.state_idx_ = state_idx_;
        centered.input_idx_ = input_idx_;
        return centered;
  }

 public:
  vector_t state_lb_;
  vector_t state_ub_;

  // Indices indicate which variables actually have constraints
  Eigen::VectorXi state_idx_;

  vector_t input_lb_;
  vector_t input_ub_;
  Eigen::VectorXi input_idx_;
};

}  // namespace ocs2
