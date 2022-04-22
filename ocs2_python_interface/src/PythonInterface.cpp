/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_python_interface/PythonInterface.h"

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::init(const RobotInterface& robot,
                           std::unique_ptr<MPC_BASE> mpcPtr) {
    if (!mpcPtr) {
        throw std::runtime_error(
            "[PythonInterface] Mpc pointer must be initialized before passing "
            "to the Python interface.");
    }
    mpcPtr_ = std::move(mpcPtr);

    mpcMrtInterface_.reset(new MPC_MRT_Interface(*mpcPtr_));

    problem_ = robot.getOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::reset(TargetTrajectories targetTrajectories) {
    targetTrajectories_ = std::move(targetTrajectories);
    mpcMrtInterface_->resetMpcNode(targetTrajectories_);
    problem_.targetTrajectoriesPtr = &targetTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setObservation(scalar_t t, Eigen::Ref<const vector_t> x,
                                     Eigen::Ref<const vector_t> u) {
    SystemObservation observation;
    observation.time = t;
    observation.state = x;
    observation.input = u;
    mpcMrtInterface_->setCurrentObservation(observation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setTargetTrajectories(
    TargetTrajectories targetTrajectories) {
    targetTrajectories_ = std::move(targetTrajectories);
    problem_.targetTrajectoriesPtr = &targetTrajectories_;
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(
        targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::advanceMpc() { mpcMrtInterface_->advanceMpc(); }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::getMpcSolution(scalar_array_t& t, vector_array_t& x,
                                     vector_array_t& u) {
    mpcMrtInterface_->updatePolicy();
    t = mpcMrtInterface_->getPolicy().timeTrajectory_;
    x = mpcMrtInterface_->getPolicy().stateTrajectory_;
    u = mpcMrtInterface_->getPolicy().inputTrajectory_;
}

void PythonInterface::evaluateMpcSolution(
    scalar_t current_time, Eigen::Ref<const vector_t> current_state,
    Eigen::Ref<vector_t> opt_state, Eigen::Ref<vector_t> opt_input) {
    // updatePolicy loads a new policy if one is available
    // evaluatePolicy computes the input given the time and state, using the
    // current policy advanceMpc actually computes the new policy; this is the
    // expensive operation
    mpcMrtInterface_->updatePolicy();
    size_t mode = 0;
    vector_t x_temp(opt_state.rows());
    vector_t u_temp(opt_input.rows());
    mpcMrtInterface_->evaluatePolicy(current_time, current_state, x_temp,
                                     u_temp, mode);
    opt_state = x_temp;
    opt_input = u_temp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::getLinearFeedbackGain(scalar_t time) {
    matrix_t K;
    mpcMrtInterface_->getLinearFeedbackGain(time, K);
    return K;
}

vector_t PythonInterface::getBias(scalar_t time) {
    vector_t bias;
    mpcMrtInterface_->getBias(time, bias);
    return bias;
}


LinearController PythonInterface::getLinearController() {
    mpcMrtInterface_->updatePolicy();
    LinearController controller;
    mpcMrtInterface_->getLinearController(controller);
    return controller;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::flowMap(scalar_t t, Eigen::Ref<const vector_t> x,
                                  Eigen::Ref<const vector_t> u) {
    return problem_.dynamicsPtr->computeFlowMap(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PythonInterface::flowMapLinearApproximation(
    scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
    return problem_.dynamicsPtr->linearApproximation(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PythonInterface::cost(scalar_t t, Eigen::Ref<const vector_t> x,
                               Eigen::Ref<const vector_t> u) {
    auto request = Request::Cost + Request::Cost + Request::SoftConstraint;
    if (penalty_ != nullptr) {
        request = request + Request::Constraint;
    }
    auto& preComputation = *problem_.preComputationPtr;
    preComputation.request(request, t, x, u);

    // get results
    scalar_t L = computeCost(problem_, t, x, u);

    if (penalty_ != nullptr) {
        const auto h =
            problem_.inequalityConstraintPtr->getValue(t, x, u, preComputation);
        SoftConstraintPenalty softConstraintPenalty(
            std::unique_ptr<PenaltyBase>(penalty_->clone()));
        L += softConstraintPenalty.getValue(t, h);
    }

    return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation
PythonInterface::costQuadraticApproximation(scalar_t t,
                                            Eigen::Ref<const vector_t> x,
                                            Eigen::Ref<const vector_t> u) {
    auto request = Request::Cost + Request::Cost + Request::SoftConstraint +
                   Request::Approximation;
    if (penalty_ != nullptr) {
        request = request + Request::Constraint;
    }
    auto& preComputation = *problem_.preComputationPtr;
    preComputation.request(request, t, x, u);

    // get results
    auto cost = approximateCost(problem_, t, x, u);

    if (penalty_ != nullptr) {
        const auto h =
            problem_.inequalityConstraintPtr->getQuadraticApproximation(
                t, x, u, preComputation);
        SoftConstraintPenalty softConstraintPenalty(
            std::unique_ptr<PenaltyBase>(penalty_->clone()));
        cost += softConstraintPenalty.getQuadraticApproximation(t, h);
    }

    return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PythonInterface::valueFunction(scalar_t t,
                                        Eigen::Ref<const vector_t> x) {
    return mpcMrtInterface_->getValueFunction(t, x).f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::valueFunctionStateDerivative(
    scalar_t t, Eigen::Ref<const vector_t> x) {
    return mpcMrtInterface_->getValueFunction(t, x).dfdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::stateInputEqualityConstraint(
    scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
    problem_.preComputationPtr->request(Request::Constraint, t, x, u);
    return problem_.equalityConstraintPtr->getValue(
        t, x, u, *problem_.preComputationPtr);
}

// Get the value of the constraint underlying a soft state-input inequality
// constraint by name
vector_t PythonInterface::softStateInputInequalityConstraint(
    const std::string& name, scalar_t t, Eigen::Ref<const vector_t> x,
    Eigen::Ref<const vector_t> u) {
    problem_.preComputationPtr->request(Request::Constraint, t, x, u);
    return dynamic_cast<StateInputSoftConstraint*>(
               &problem_.softConstraintPtr->get(name))
        ->get()
        .getValue(t, x, u, *problem_.preComputationPtr);
}

// Get the value of a hard state-input inequality constraint by name
vector_t PythonInterface::stateInputInequalityConstraint(
    const std::string& name, scalar_t t, Eigen::Ref<const vector_t> x,
    Eigen::Ref<const vector_t> u) {
    problem_.preComputationPtr->request(Request::Constraint, t, x, u);
    return problem_.inequalityConstraintPtr->get(name).getValue(t, x, u, *problem_.preComputationPtr);
}

vector_t PythonInterface::stateInequalityConstraint(
    const std::string& name, scalar_t t, Eigen::Ref<const vector_t> x) {
    return dynamic_cast<StateSoftConstraint*>(
               &problem_.stateSoftConstraintPtr->get(name))
        ->get()
        .getValue(t, x, *problem_.preComputationPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation
PythonInterface::stateInputEqualityConstraintLinearApproximation(
    scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
    problem_.preComputationPtr->request(
        Request::Constraint + Request::Approximation, t, x, u);
    return problem_.equalityConstraintPtr->getLinearApproximation(
        t, x, u, *problem_.preComputationPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::stateInputEqualityConstraintLagrangian(
    scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
    vector_t zero_u = vector_t::Zero(u.rows());

    const auto g =
        stateInputEqualityConstraintLinearApproximation(t, x, zero_u);
    const matrix_t& Dm = g.dfdu;
    const vector_t& c = g.f;

    const auto Phi = costQuadraticApproximation(t, x, zero_u);
    const matrix_t& R = Phi.dfduu;
    const vector_t& r = Phi.dfdu;

    const matrix_t B = flowMapLinearApproximation(t, x, zero_u).dfdu;

    matrix_t RinvChol;
    LinearAlgebra::computeInverseMatrixUUT(R, RinvChol);
    matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol;
    ocs2::LinearAlgebra::computeConstraintProjection(
        Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol);

    vector_t costate = valueFunctionStateDerivative(t, x);

    return DmDager.transpose() *
           (R * DmDager * c - r - B.transpose() * costate);
}

}  // namespace ocs2
