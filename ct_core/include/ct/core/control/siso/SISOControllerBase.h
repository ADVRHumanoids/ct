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

#ifndef SISOCONTROLLERBASE_H_
#define SISOCONTROLLERBASE_H_

#include <boost/concept_check.hpp>
#include <memory>

#include <ct/core/types/Time.h>

namespace ct {
namespace core {

//! A general SISO controller interface
/*!
 * A standard interface defining a single-input single-output controller, such as a PIDController.
 * Inherit from this class to implement your custom SISO controller.
 */
class SISOControllerBase
{
public:
	//! Default constructor
	SISOControllerBase() {};

	//! Copy constructor
	SISOControllerBase(const SISOControllerBase& arg) {}

	//! Destructor
	virtual ~SISOControllerBase() {};

	//! Deep cloning destructor
	/*!
	 * Needs to be implemented by derived class.
	 * @return pointer to cloned instance
	 */
	virtual SISOControllerBase* clone() const = 0;

	//! Computes the control action
	/*!
	 * Takes the current state and time and computes the corresponding output.
	 * Needs to be implemented by any derived class.
	 * @param state current state
	 * @param t current time
	 * @return resulting control action
	 */
	virtual double computeControl(const double& state, const core::Time& t) = 0;

protected:


};

} // namespace core
} // namespace ct



#endif /* SISOCONTROLLERBASE_H_ */
