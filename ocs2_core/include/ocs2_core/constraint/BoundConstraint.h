#pragma once

#include <ocs2_core/Types.h>

namespace ocs2 {

// TODO this should just be the bounds, no booleans

/**
 * Bound constraint
 */
class BoundConstraint {
 public:
  BoundConstraint() {}

  BoundConstraint* clone() const {
    return new BoundConstraint(*this);
  }

  bool empty() const {
    return state_idx_.size() == 0 && input_idx_.size() == 0;
  }

  void setZero(size_t state_dim, size_t input_dim) {
    state_lb_.setZero(state_dim);
    state_ub_.setZero(state_dim);
    input_lb_.setZero(input_dim);
    input_ub_.setZero(input_dim);
  }

  vector_t violation(const vector_t& state, const vector_t& input) const {
    vector_t v(2 * state.size() + 2 * input.size());
    v << state_ub_ - state, state - state_lb_, input_ub_ - input, input - input_lb_;
    return v;
  }

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

  // BoundConstraint(size_t state_dim, size_t input_dim) {
  //   // Initialize sizes and disable all constraints
  //   state_lb_.setZero(state_dim);
  //   state_ub_.setZero(state_dim);
  //   state_bounded_ = (state_dim > 0);
  //   // state_mask_.resize(state_dim, false);
  //   // state_lb_mask_.setZero(state_dim);
  //   // state_ub_mask_.setZero(state_dim);
  //
  //   input_lb_.setZero(input_dim);
  //   input_ub_.setZero(input_dim);
  //   input_bounded_ = (input_dim > 0);
  //   // input_mask_.resize(input_dim, false);
  //   // input_lb_mask_.setZero(input_dim);
  //   // input_ub_mask_.setZero(input_dim);
  // }

 public:
  vector_t state_lb_;
  vector_t state_ub_;
  Eigen::VectorXi state_idx_;
  // Mask being 1 (true) indicates the constraint is active
  // std::vector<bool> state_mask_;
  // bool state_bounded_ = false;

  vector_t input_lb_;
  vector_t input_ub_;
  Eigen::VectorXi input_idx_;
  // std::vector<bool> input_mask_;
  // bool input_bounded_ = false;
};

}  // namespace ocs2
