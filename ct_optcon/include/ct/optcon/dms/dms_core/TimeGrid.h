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


#ifndef CT_OPTCON_DMS_CORE_TIME_GRID_H_
#define CT_OPTCON_DMS_CORE_TIME_GRID_H_

//#define DEBUG_TIMEGRID

#include <math.h>
#include <cmath>
#include <ct/core/core.h>

#include <ct/core/Types>

namespace ct {
namespace optcon {

/**
 * @ingroup    DMS
 *
 * This class describes the time-grid underlying the shots in the dms problem In
 * total, we have N+1 pairs of (s_i, q_i) and N shots between them.
 *
 * We assume that starting time t_0 = 0.0 [sec]
 */
class TimeGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TimeGrid() = delete;

	/**
	 * @brief      Custom constructor
	 *
	 * @param[in]  numberOfShots  The number of shots
	 * @param[in]  timeHorizon    The dms time horizon
	 */
	TimeGrid(const size_t numberOfShots, const double timeHorizon):
	numberOfShots_(numberOfShots),
	timeHorizon_(timeHorizon),
	t_(numberOfShots + 1, 0.0)
	{
		makeUniformGrid();
	}

	/**
	 * @brief      Updates the timegrid when the number of shots changes
	 *
	 * @param[in]  numberOfShots  The new number of shots
	 */
	void changeShotCount(const size_t numberOfShots)
	{
		numberOfShots_ = numberOfShots;
		t_.clear();
		t_.resize(numberOfShots_ + 1, 0.0);
		makeUniformGrid();
	}

	/**
	 * @brief      Updates the timegrid when the timehorizon changes
	 *
	 * @param[in]  timeHorizon  The new time horizon
	 */
	void changeTimeHorizon(const double timeHorizon)
	{
		timeHorizon_ = timeHorizon;
		makeUniformGrid();
	}



	/**
	 * @brief      This method updates the timegrid when new optimized time
	 *             segments arrive from the nlp solver. Only gets called when
	 *             using timegrid optimization, otherwise the timegrid stays
	 *             fixed
	 *
	 * @param[in]  h_segment  The vector of the new optimized time segments
	 */
	void updateTimeGrid(const Eigen::VectorXd& h_segment)
	{
		t_[0] = 0.0; //by convention (just for documentation)

		for(size_t i = 0; i < (size_t) h_segment.size(); ++i)
			t_[i+1] = t_[i] + h_segment(i);

#ifdef DEBUG_TIMEGRID
		std::cout << " ... in updateTimeGrid(). t_ =  ";
		for(size_t i = 0; i<t_.size(); i++)
			std::cout << std::setprecision (10) <<t_[i] << "  ";

		std::cout << std::endl;
#endif
	}


	/**
	 * @brief      Creates a uniform timegrid
	 */
	void makeUniformGrid()
	{
		for (size_t i = 0; i < numberOfShots_ + 1; i++)
			t_[i]	= i * (double)(timeHorizon_/ (double)numberOfShots_);
	}


	/**
	 * @brief      Returns to start time of a shot
	 *
	 * @param[in]  shot_index  The shot number
	 *
	 * @return     The start time
	 */
	const ct::core::Time getShotStartTime(const size_t shot_index) const{
		return t_[shot_index];
	}

	/**
	 * @brief      Returns the end time of a shot
	 *
	 * @param[in]  shot_index  The shot number
	 *
	 * @return     The end time
	 */
	const ct::core::Time getShotEndTime(const size_t shot_index) const{
		return t_[shot_index+1];
	}

	/**
	 * @brief      Returns to duration of a shot
	 *
	 * @param[in]  shot_index  The shot index
	 *
	 * @return     The duration
	 */
	const ct::core::Time getShotDuration(const size_t shot_index) const{
		return (t_[shot_index+1]-t_[shot_index]);
	}

	/**
	 * @brief      Returns the underlying TimeArray
	 *
	 * @return     The underlying TimeArray
	 */
	const ct::core::TimeArray& toImplementation()
	{
		return t_;
	}



	/**
	 * @brief      Returns the initial timehorizon of the problem
	 *
	 * @return     The initial time horizon
	 */
	const ct::core::Time getTimeHorizon() const
	{
		return timeHorizon_;
	}

	/**
	 * @brief      Returns the optimized timehorizon
	 *
	 * @return     The optimized timehorizon
	 */
	const ct::core::Time getOptimizedTimeHorizon() const
	{
		return t_.back();
	}


private:
	size_t numberOfShots_;
	double timeHorizon_;

	// the individual times of each pair from i=0,..., N
	ct::core::TimeArray t_;
};

} // namespace optcon
} // namespace ct


#endif //CT_OPTCON_DMS_CORE_TIME_GRID_H_
