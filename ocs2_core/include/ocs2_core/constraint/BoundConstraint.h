#pragma once

#include <numeric>

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Bound constraint
 */
class BoundConstraint {
   public:
    BoundConstraint() {}

    BoundConstraint(const vector_t& state_lb, const vector_t& state_ub,
                    const vector_t& input_lb, const vector_t& input_ub)
        : state_lb_(state_lb),
          state_ub_(state_ub),
          input_lb_(input_lb),
          input_ub_(input_ub) {}

    BoundConstraint* clone() const { return new BoundConstraint(*this); }

    // True if no bound constraints are active on either state or input
    bool empty() const {
        return numStateConstraints() == 0 && numInputConstraints() == 0;
    }

    void setZero(size_t state_dim, size_t input_dim) {
        state_lb_.setZero(state_dim);
        state_ub_.setZero(state_dim);
        input_lb_.setZero(input_dim);
        input_ub_.setZero(input_dim);
    }

    void setStateIndices(const std::vector<int>& state_idx) {
        const size_t n = state_lb_.size();
        for (int idx : state_idx) {
            if (idx < 0 || idx >= n) {
                throw std::runtime_error("State box constraint indices outside of range.");
            }
        }
        state_idx_ = state_idx;

        // TODO we can build a selector matrix to simplify computation of the
        // violation
        // matrix_t Sx = matrix_t::Zero(numStateConstraints(), state_lb_.size());
        // for (int i = 0; i < numStateConstraints(); ++i) {
        //     Sx(i, state_idx[i]) = 1;
        // }
    }

    void setStateIndices(int start, size_t size) {
        std::vector<int> state_idx(size);
        std::iota(std::begin(state_idx), std::end(state_idx), start);
        setStateIndices(state_idx);
    }

    void setInputIndices(const std::vector<int>& input_idx) {
        const size_t n = input_lb_.size();
        for (int idx : input_idx) {
            if (idx < 0 || idx >= n) {
                throw std::runtime_error("Input box constraint indices outside of range.");
            }
        }
        input_idx_ = input_idx;
    }

    void setInputIndices(int start, size_t size) {
        std::vector<int> input_idx(size);
        std::iota(std::begin(input_idx), std::end(input_idx), start);
        setInputIndices(input_idx);
    }

    size_t numStateConstraints() const {
        return state_idx_.size();
    }

    size_t numInputConstraints() const {
        return input_idx_.size();
    }

    // Get the constraint violation: a negative value has violated the
    // constraint
    vector_t violation(const vector_t& state, const vector_t& input) const {
        const size_t nx = numStateConstraints();
        const size_t nu = numInputConstraints();
        vector_t v(2 * nx + 2 * nu);
        for (int i = 0; i < nx; ++i) {
            int idx = state_idx_[i];
            v(i) = state_ub_(idx) - state(idx);
            v(nx + i) = state(idx) - state_lb_(idx);
        }
        for (int i = 0; i < nu; ++i) {
            int idx = input_idx_[i];
            v(2 * nx + i) = input_ub_(idx) - input(idx);
            v(2 * nx + nu + i) = input(idx) - input_lb_(idx);
        }
        return v;
    }

    // Center the constraints around a given state and input
    BoundConstraint center(const vector_t& state, const vector_t& input) const {
      // TODO this fundamentally assumes that hpipm can take as long a vector as
      // needed
      BoundConstraint centered;
      centered.state_lb_ = state_lb_ - state;
      centered.state_ub_ = state_ub_ - state;
      centered.input_lb_ = input_lb_ - input;
      centered.input_ub_ = input_ub_ - input;
      centered.state_idx_ = state_idx_;
      centered.input_idx_ = input_idx_;
      return centered;
    }

    // We leave the member variables public since we need to access their data
    // for solving
   public:
    vector_t state_lb_;
    vector_t state_ub_;

    // Indices indicate which variables actually have constraints
    std::vector<int> state_idx_;

    vector_t input_lb_;
    vector_t input_ub_;
    std::vector<int> input_idx_;
};

}  // namespace ocs2
