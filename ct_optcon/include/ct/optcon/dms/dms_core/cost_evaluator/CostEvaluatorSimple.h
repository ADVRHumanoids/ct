/***********************************************************************************
Copyright (c) 2017, ETH Zurich, Google Inc. All rights reserved.
Authors: Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. 

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef CT_OPTCON_DMS_DMS_CORE_COST_EVALUATOR_SIMPLE_H_
#define CT_OPTCON_DMS_DMS_CORE_COST_EVALUATOR_SIMPLE_H_

#include <omp.h>
#include <math.h>
#include <cmath>

#include <ct/optcon/costfunction/CostFunctionQuadratic.hpp>

#include <ct/optcon/dms/dms_core/OptVectorDms.h>
#include <ct/optcon/dms/dms_core/spline/SplinerBase.h>
#include <ct/optcon/nlp/DiscreteCostEvaluatorBase.h>

#include <ct/core/types/ControlVector.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/types/Time.h>

namespace ct {
namespace optcon {


/**
 * @ingroup    DMS
 *
 * @brief      Evaluates the cost at the shots and performs some interpolation
 *             in between
 *
 * @tparam     STATE_DIM    The state dimension
 * @tparam     CONTROL_DIM  The control dimension
 */
template <size_t STATE_DIM, size_t CONTROL_DIM>
class CostEvaluatorSimple : public DiscreteCostEvaluatorBase
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DmsDimensions<STATE_DIM, CONTROL_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;

	CostEvaluatorSimple() = delete;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  costFct   The cost function
	 * @param[in]  w         The optimization variables
	 * @param[in]  timeGrid  The time grid
	 * @param[in]  settings  The dms settings
	 */
	CostEvaluatorSimple(
			std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct,
			std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w,
			std::shared_ptr<TimeGrid> timeGrid,
			DmsSettings settings):
				costFct_(costFct),
				w_(w),
				timeGrid_(timeGrid),
				settings_(settings)
	{
		phi_.resize(settings_.N_+1);
		phi_diff_h_ = Eigen::VectorXd::Ones(settings_.N_+1, 1);
		updatePhi();
	}

	virtual ~CostEvaluatorSimple(){}

	virtual double eval() override;

	virtual void evalGradient(size_t grad_length, Eigen::Map<Eigen::VectorXd>& grad) override;

private:

	/**
	 * @brief      Updates the weights for the cost interpolation
	 */
	void updatePhi();

	std::shared_ptr<ct::optcon::CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFct_;
	std::shared_ptr<OptVectorDms<STATE_DIM, CONTROL_DIM>> w_;
	std::shared_ptr<TimeGrid> timeGrid_;
	const DmsSettings settings_;
	Eigen::VectorXd phi_; /* the summation weights */
	Eigen::VectorXd phi_diff_h_; /* the summation weights for derivative w.r.t h */
};



template <size_t STATE_DIM, size_t CONTROL_DIM>
double CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>::eval()
{
	updatePhi();
	double cost = 0.0;
	// go through all shots and eval cost at nodes
	for (size_t i = 0; i < settings_.N_ + 1; ++i)
	{
		costFct_->setCurrentStateAndControl(w_->getOptimizedState(i), w_->getOptimizedControl(i));
		cost += phi_(i)*costFct_->evaluateIntermediate();
	}

	costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
	cost += costFct_->evaluateTerminal();
	assert(cost == cost);
	return cost;
}



template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>::evalGradient(size_t grad_length, Eigen::Map<Eigen::VectorXd>& grad)
{
	updatePhi();
	grad.setZero();
	// Loop only until N, terminalCost will be added afterwards?
	for (size_t i = 0; i< settings_.N_ + 1; ++i)
	{
		costFct_->setCurrentStateAndControl(w_->getOptimizedState(i), w_->getOptimizedControl(i));
		grad.segment(w_->getStateIndex(i), STATE_DIM) += phi_(i) * costFct_->stateDerivativeIntermediate();
		grad.segment(w_->getControlIndex(i), CONTROL_DIM) += phi_(i)*costFct_->controlDerivativeIntermediate() ;

		if(settings_.objectiveType_ == DmsSettings::OPTIMIZE_GRID && i < settings_.N_)
		{
			if(settings_.splineType_ == DmsSettings::ZERO_ORDER_HOLD)
			{
				costFct_->setCurrentStateAndControl(w_->getOptimizedState(i), w_->getOptimizedControl(i));
				double dJdH = phi_diff_h_(i) * costFct_->evaluateIntermediate();
				size_t idx = w_->getTimeSegmentIndex(i);
				grad(idx) = dJdH;
			}

			else if(settings_.splineType_ == DmsSettings::PIECEWISE_LINEAR)
			{
				costFct_->setCurrentStateAndControl(w_->getOptimizedState(i), w_->getOptimizedControl(i));
				double dJdH = 0.5 * costFct_->evaluateIntermediate();
				costFct_->setCurrentStateAndControl(w_->getOptimizedState(i+1), w_->getOptimizedControl(i+1));
				dJdH += 0.5 * costFct_->evaluateIntermediate();
				grad(w_->getTimeSegmentIndex(i)) = dJdH;
			}
		}	
	}

	/* gradient of terminal cost */
	costFct_->setCurrentStateAndControl(w_->getOptimizedState(settings_.N_), control_vector_t::Zero());
	grad.segment(w_->getStateIndex(settings_.N_), STATE_DIM) += costFct_->stateDerivativeTerminal();
}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void CostEvaluatorSimple<STATE_DIM, CONTROL_DIM>::updatePhi()
	{
		switch (settings_.splineType_)
		{
		case DmsSettings::ZERO_ORDER_HOLD:
		{
			// set weights
			for(size_t i = 0; i < settings_.N_; i++)
				phi_(i) = (timeGrid_->getShotDuration(i)); // / curr_total_time;

			// set weight for s_N and q_N to zero
			phi_(settings_.N_) = 0.0;
			// phi_diff_h_(settings_.N_) = 0.0;

			break;
		}
		case DmsSettings::PIECEWISE_LINEAR:
		{
			// set special weight for s_0
			phi_(0) = 0.5 * (timeGrid_->getShotDuration(0)); // / curr_total_time;

			// set weights for s_1, ..., s_(N-1)
			for(size_t i = 1; i < settings_.N_; i++)
				phi_(i) = 0.5 * (timeGrid_->getShotDuration(i) + timeGrid_->getShotDuration(i-1));

			// set special weight for s_N
			phi_(settings_.N_) = 0.5 * (timeGrid_->getShotDuration(settings_.N_-1)) ; // / curr_total_time;

			break;

		}
		default:
			throw(std::runtime_error(" ERROR: Unknown spline-type in CostEvaluatorSimple - exiting."));
		}
	}

} // namespace optcon
} // namespace ct

#endif
