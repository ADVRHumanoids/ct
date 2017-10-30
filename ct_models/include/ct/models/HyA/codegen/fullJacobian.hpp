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

/*! \file
 *	\brief		Auto-generated code for computing the derivative of qdd with respect to state and input
 *  \author	    Michael Neunert
 *
 *  @example 	timingFullvsSeparateJacobian.cpp
 *  This is an example of how to use the generated code.
 *
 *  @example 	timingFullJacobian.cpp
 *  This is an example of how to use the generated code.
 */

#ifndef HYA_CODEGEN_DERIVATIVE_FULL_JACOBIAN_HPP_
#define HYA_CODEGEN_DERIVATIVE_FULL_JACOBIAN_HPP_

#include <array>
#include <Eigen/Core>

namespace ct_HyA {

Eigen::Matrix<double, 12+6, 6> computeFullJacobianCodegen(const Eigen::Matrix<double, 12, 1>& state, const Eigen::Matrix<double, 6, 1>& tau);

}

#endif /* TEST_CODEGEN_DERIVATIVE_FULL_JACOBIAN_HPP_ */
