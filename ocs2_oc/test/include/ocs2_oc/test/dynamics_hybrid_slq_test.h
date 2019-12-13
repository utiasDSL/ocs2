/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>

enum { STATE_DIM = 3, INPUT_DIM = 1 };

namespace ocs2 {

// #######################
// ####LOGIC CLASSES######
// #######################
class hybridSysLogic final : public HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = HybridLogicRules;

  hybridSysLogic() = default;

  ~hybridSysLogic() override = default;

  hybridSysLogic(scalar_array_t switchingTimes, size_array_t subsystemsSequence)
      : BASE(std::move(switchingTimes), std::move(subsystemsSequence)) {}

  void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) override {}

  void update() override {}

 protected:
  void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                  const scalar_t& finalTime) override{};
};

// #######################
// ###DYNAMICS CLASSES####
// #######################
class hybridSysDynamics1 final : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hybridSysDynamics1() = default;
  ~hybridSysDynamics1() override = default;

  void computeFlowMap(const double& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    A << -0.1, 0.9, 0.0, -1, -0.01, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, STATE_DIM, INPUT_DIM> B;
    B << 0.0, 1.0, 0.0;
    Eigen::Matrix<double, STATE_DIM, 1> F;
    F << 0.0, 0.0, 0.0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 1;
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = 1;
    guardSurfacesValue[1] = -state[0] * state[1];
  }

  hybridSysDynamics1* clone() const override { return new hybridSysDynamics1(*this); }
};

class hybridSysDynamics2 final : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hybridSysDynamics2() = default;
  ~hybridSysDynamics2() override = default;

  void computeFlowMap(const double& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, STATE_DIM, INPUT_DIM> B;
    B << 0.0, 1.0, 0.0;
    Eigen::Matrix<double, STATE_DIM, 1> F;
    F << 0.0, 0.0, 0.0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 0;
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = state[0] * state[1];
    guardSurfacesValue[1] = 1;
  }

  hybridSysDynamics2* clone() const final { return new hybridSysDynamics2(*this); }
};

class hybridSysDynamics final : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = ControlledSystemBase<STATE_DIM, INPUT_DIM>;

  hybridSysDynamics() : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(new hybridSysDynamics1);
    subsystemDynamicsPtr_[1].reset(new hybridSysDynamics2);
  }

  ~hybridSysDynamics() override = default;

  hybridSysDynamics* clone() const override { return new hybridSysDynamics(*this); }

  hybridSysDynamics(const hybridSysDynamics& other) : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
    subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
  }

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    size_t activeSubsystem = x[2];
    subsystemDynamicsPtr_[activeSubsystem]->computeFlowMap(t, x, u, dxdt);
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    size_t activeSubsystem = state[2];
    subsystemDynamicsPtr_[activeSubsystem]->computeJumpMap(time, state, mappedState);
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    size_t activeSubsystem = state[2];
    guardSurfacesValue = Eigen::Matrix<double, 2, 1>();
    subsystemDynamicsPtr_[activeSubsystem]->computeGuardSurfaces(time, state, guardSurfacesValue);
  }

 private:
  std::vector<Base::Ptr> subsystemDynamicsPtr_;
};

// ############################
// ####DERIVATIVE CLASSES######
// ############################
class hybridSysDerivatives1 final : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hybridSysDerivatives1() = default;
  ~hybridSysDerivatives1() override = default;

  void getFlowMapDerivativeState(state_matrix_t& A) override { A << -0.1, 0.9, 0.0, -1.0, -0.01, 0.0, 0.0, 0.0, 0.0; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B << 0.0, 1.0, 0.0; }

  hybridSysDerivatives1* clone() const override { return new hybridSysDerivatives1(*this); }
};

class hybridSysDerivatives2 final : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hybridSysDerivatives2() = default;
  ~hybridSysDerivatives2() override = default;

  void getFlowMapDerivativeState(state_matrix_t& A) override { A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B << 0.0, 1.0, 0.0; }

  hybridSysDerivatives2* clone() const override { return new hybridSysDerivatives2(*this); }
};

class hybridSysDerivatives final : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = DerivativesBase<STATE_DIM, INPUT_DIM>;

  hybridSysDerivatives() : activeSubsystem_(1), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(new hybridSysDerivatives1);
    subsystemDerPtr_[1].reset(new hybridSysDerivatives2);
  }

  ~hybridSysDerivatives() override = default;

  hybridSysDerivatives* clone() const final { return new hybridSysDerivatives(*this); }

  hybridSysDerivatives(const hybridSysDerivatives& other) : activeSubsystem_(other.activeSubsystem_), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(other.subsystemDerPtr_[0]->clone());
    subsystemDerPtr_[1].reset(other.subsystemDerPtr_[1]->clone());
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    Base::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = x[2];
    subsystemDerPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(state_matrix_t& A) override { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B); }

 private:
  int activeSubsystem_;
  std::vector<Base::Ptr> subsystemDerPtr_;
};

// #######################
// #### COST CLASSES######
// #######################
class system_cost_1 final : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_cost_1() = default;
  ~system_cost_1() = default;

  system_cost_1* clone() const override { return new system_cost_1(*this); }

  void getIntermediateCost(scalar_t& L) override { L = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2) + 0.005 * pow(u_[0], 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override { dLdx << x_[0], x_[1], 0.0; }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override { dLdxx << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override { dLdu << 0.01 * u_[0]; }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override { dLduu << 0.01; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) override { dLdxu.setZero(); }

  /*
  Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) override { Phi = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override { dPhidx << x_[0], x_[1], 0.0; }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override { dPhidxx << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; }
};

class system_cost_2 final : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_cost_2() = default;
  ~system_cost_2() override = default;

  system_cost_2* clone() const override { return new system_cost_2(*this); }

  void getIntermediateCost(scalar_t& L) override { L = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2) + 0.005 * pow(u_[0], 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override { dLdx << x_[0], x_[1], 0.0; }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override { dLdxx << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override { dLdu << 0.01 * u_[0]; }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override { dLduu << 0.01; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) override { dLdxu.setZero(); }

  /*
  Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) override { Phi = 0.5 * pow(x_[0], 2) * pow(x_[1], 2); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override { dPhidx << x_[0], x_[1], 0.0; }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override { dPhidxx << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; }
};

class system_cost final : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = CostFunctionBase<STATE_DIM, INPUT_DIM>;

  system_cost() : activeSubsystem_(1), subsystemCostPtr_(2) {
    subsystemCostPtr_[0].reset(new system_cost_1);
    subsystemCostPtr_[1].reset(new system_cost_2);
  }

  system_cost* clone() const final { return new system_cost(*this); }

  ~system_cost() override = default;

  system_cost(const system_cost& other) : activeSubsystem_(other.activeSubsystem_), subsystemCostPtr_(2) {
    subsystemCostPtr_[0].reset(other.subsystemCostPtr_[0]->clone());
    subsystemCostPtr_[1].reset(other.subsystemCostPtr_[1]->clone());
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    t_ = t;
    x_ = x;
    u_ = u;

    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }
  /*
   * Intermediate cost function
   */

  void getIntermediateCost(scalar_t& L) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCost(L);
  }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeState(dLdx);
  }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeState(dLdxx);
  }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeInput(dLdu);
  }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeInput(dLduu);
  }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeInputState(dLdxu);
  }

  /*
          Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getTerminalCost(Phi);
  }

  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getTerminalCostDerivativeState(dPhidx);
  }

  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override {
    activeSubsystem_ = x_[2];
    subsystemCostPtr_[activeSubsystem_]->getTerminalCostSecondDerivativeState(dPhidxx);
  }

 private:
  int activeSubsystem_;
  std::vector<Base::Ptr> subsystemCostPtr_;
};

// #############################
// #### CONSTRAINT CLASSES######
// #############################
class hybridSysConstraints1 final : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  hybridSysConstraints1() = default;
  ~hybridSysConstraints1() = default;

  hybridSysConstraints1* clone() const override { return new hybridSysConstraints1(*this); }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero();
    dhdx[1].setZero();
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3] << -1.0, 0.0, 0.0;
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0] << -1.0;
    dhdu[1] << 1.0;
    dhdu[2] << 0.0;
    dhdu[3] << 0.0;
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero();
    ddhdxdx[1].setZero();
    ddhdxdx[2].setZero();
    ddhdxdx[3].setZero();
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero();
    ddhdudu[1].setZero();
    ddhdudu[2].setZero();
    ddhdudu[3].setZero();
  }
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero();
    ddhdudx[1].setZero();
    ddhdudx[2].setZero();
    ddhdudx[3].setZero();
  }
};

class hybridSysConstraints2 final : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  hybridSysConstraints2() = default;
  ~hybridSysConstraints2() override = default;

  hybridSysConstraints2* clone() const override { return new hybridSysConstraints2(*this); }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero();
    dhdx[1].setZero();
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3] << -1.0, 0.0, 0.0;
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0] << -1.0;
    dhdu[1] << 1.0;
    dhdu[2] << 0.0;
    dhdu[3] << 0.0;
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero();
    ddhdxdx[1].setZero();
    ddhdxdx[2].setZero();
    ddhdxdx[3].setZero();
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero();
    ddhdudu[1].setZero();
    ddhdudu[2].setZero();
    ddhdudu[3].setZero();
  }
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero();
    ddhdudx[1].setZero();
    ddhdudx[2].setZero();
    ddhdudx[3].setZero();
  }
};

class hybridSysConstraints final : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  using Base = ConstraintBase<STATE_DIM, INPUT_DIM>;

  hybridSysConstraints() : activeSubsystem_(1), subsystemConstPtr_(2) {
    subsystemConstPtr_[0].reset(new hybridSysConstraints1);
    subsystemConstPtr_[1].reset(new hybridSysConstraints2);
  }

  ~hybridSysConstraints() override = default;

  virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    t_ = t;
    x_ = x;
    u_ = u;

    activeSubsystem_ = x[2];
    subsystemConstPtr_[activeSubsystem_]->setCurrentStateAndControl(t_, x_, u_);
  }

  void getInequalityConstraint(scalar_array_t& h) override { subsystemConstPtr_[activeSubsystem_]->getInequalityConstraint(h); }

  size_t numInequalityConstraint(const scalar_t& time) override {
    return subsystemConstPtr_[activeSubsystem_]->numInequalityConstraint(time);
  }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesState(dhdx);
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInput(dhdu);
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  }

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInputState(ddhdudx);
  }

  hybridSysConstraints* clone() const override { return new hybridSysConstraints(*this); }

 private:
  int activeSubsystem_;
  std::vector<Base::Ptr> subsystemConstPtr_;
};

using system_op = SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
}  // namespace ocs2
