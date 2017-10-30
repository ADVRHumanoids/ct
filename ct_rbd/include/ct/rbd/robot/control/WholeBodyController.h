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

#ifndef WholeBodyController_H_
#define WholeBodyController_H_

#include <ct/rbd/rbd.h>

#include "JointPositionController.h"

namespace ct {
namespace rbd {

/**
 * @class WholeBodyController
 */
template <size_t NJOINTS>
class WholeBodyController : public ct::core::Controller<2*6 + 2*NJOINTS, NJOINTS>
{
public:
	static const size_t STATE_DIM = 2*6 + 2*NJOINTS;

	WholeBodyController() {};
	virtual ~WholeBodyController() {};

	virtual WholeBodyController<NJOINTS>* clone() const override { throw std::runtime_error("Not implemented"); };

	virtual void computeControl(const core::StateVector<STATE_DIM>& state, const core::Time& t, core::ControlVector<NJOINTS>& control) override
	{
		ct::rbd::RBDState<NJOINTS> x;
		x.fromStateVectorEulerXyz(state);
		core::StateVector<2*NJOINTS> jState = x.joints().toImplementation();

		jointController_.computeControl(jState, t, control);
	}

	JointPositionController<NJOINTS>& getJointController() { return jointController_; }


protected:
	JointPositionController<NJOINTS> jointController_;

};

} // namespace rbd
} // namespace ct



#endif /* WholeBodyController_H_ */
