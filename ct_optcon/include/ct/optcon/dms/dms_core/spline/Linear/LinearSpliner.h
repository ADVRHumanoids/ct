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

#ifndef CT_OPTCON_DMS_CORE_LINEAR_SPLINER_DMS_H_
#define CT_OPTCON_DMS_CORE_LINEAR_SPLINER_DMS_H_

#include "ct/optcon/dms/dms_core/spline/SplinerBase.h"
#include <ct/optcon/dms/dms_core/TimeGrid.h>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * @brief      The linear spline implementation
 *
 * @tparam     T     The vector type to be splined
 */
template<class T>
class LinearSpliner : public SplinerBase<T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef T vector_t;
	typedef std::vector<vector_t, Eigen::aligned_allocator<vector_t>> vector_array_t;
	typedef Eigen::Matrix<double, T::DIM, T::DIM> matrix_t;

	LinearSpliner() = delete;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  grid  The dms timegrid
	 */
	LinearSpliner(std::shared_ptr<TimeGrid> grid):
		timeGrid_(grid)
	{}

	virtual ~LinearSpliner(){}

	void computeSpline(const vector_array_t& points) override {
		nodes_ = points;
	}

	// evaluate spline and return vector at interpolation time
	virtual vector_t evalSpline (const double time,  const size_t shotIdx) override {

		Eigen::VectorXd result;
		result.resize(T::DIM);

		//		int shotIdx = timeGrid_->getShotIndex(time);
		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */
		double t_s_end = timeGrid_->getShotEndTime(shotIdx);		/* time when this particular shot ends */

		assert(shotIdx < nodes_.size());

		result = nodes_[shotIdx]*(t_s_end - time) / t_shot
				+ nodes_[shotIdx+1]*(time-t_s_start) / t_shot;

		return result;
	}


	virtual vector_t splineDerivative_t (const double time,  const size_t shotIdx) const override {

		vector_t result;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/

		result = (nodes_[shotIdx+1]- nodes_[shotIdx]) / t_shot;

		return result;
	}


	virtual vector_t splineDerivative_h_i (const double time, const size_t shotIdx) const override {

		vector_t result;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a whole shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */

		result = (time-t_s_start)*(nodes_[shotIdx] - nodes_[shotIdx + 1])/(t_shot*t_shot);

		return result;
	}

	virtual matrix_t splineDerivative_q_i (const double time,  const size_t shotIdx) const override {

		matrix_t drv;

		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of a the shot*/
		double t_s_end = timeGrid_->getShotEndTime(shotIdx);		/* time when this particular shot ends */

		drv.setIdentity();
		drv *= (t_s_end - time) / t_shot;

		return drv;
	}


	virtual matrix_t splineDerivative_q_iplus1(const double time,  const size_t shotIdx) const override {

		matrix_t drv;

		//		int shotIdx = timeGrid_->getShotIndex(time);
		double t_shot = timeGrid_->getShotDuration(shotIdx);		/* current duration of the shot*/
		double t_s_start = timeGrid_->getShotStartTime(shotIdx);	/* time when this particular shot started */

		drv.setIdentity();
		drv *= (time-t_s_start) / t_shot;

		return drv;
	}


private:

	vector_array_t nodes_;	// an array of references to grid points between which is interpolated

	std::shared_ptr<TimeGrid> timeGrid_;

};

} // namespace optcon
} // namespace ct

#endif
