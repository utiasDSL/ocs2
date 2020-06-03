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
#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsSystem final : public SystemDynamicsBase {
 public:
  CircularKinematicsSystem() : SystemDynamicsBase(2, 2){};
  ~CircularKinematicsSystem() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) override { return u; }

  matrix_t getFlowMapDerivativeState() override { return matrix_t::Zero(2, 2); }

  matrix_t getFlowMapDerivativeInput() override { return matrix_t::Identity(2, 2); }

  CircularKinematicsSystem* clone() const override { return new CircularKinematicsSystem(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsCost final : public CostFunctionBaseAD {
 public:
  CircularKinematicsCost() : CostFunctionBaseAD(2, 2) {}
  ~CircularKinematicsCost() override = default;

  CircularKinematicsCost* clone() const override { return new CircularKinematicsCost(*this); }

 protected:
  ad_scalar_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                       const ad_vector_t& parameters) const override {
    return 0.5 * pow(state(0) * input(1) - state(1) * input(0) - 1.0, 2) + 0.005 * input.dot(input);
  }

  ad_scalar_t terminalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override {
    return ad_scalar_t(0.0);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsConstraints final : public ConstraintBase {
 public:
  CircularKinematicsConstraints() : ConstraintBase(2, 2) {}
  ~CircularKinematicsConstraints() override = default;

  CircularKinematicsConstraints* clone() const override { return new CircularKinematicsConstraints(*this); }

  vector_t getStateInputEqualityConstraint() override {
    vector_t e(1);
    e(0) = x_.dot(u_);
    return e;
  }

  matrix_t getStateInputEqualityConstraintDerivativesState() override {
    matrix_t C(1, 2);
    C = u_.transpose();
    return C;
  }

  matrix_t getStateInputEqualityConstraintDerivativesInput() override {
    matrix_t D(1, 2);
    D = x_.transpose();
    return D;
  }
};

}  // namespace ocs2
