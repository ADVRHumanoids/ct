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

#ifndef CT_MODELS_QUADMODELPARAMETERS_HPP_
#define CT_MODELS_QUADMODELPARAMETERS_HPP_

#include <Eigen/Dense>

namespace ct{
namespace models{
namespace quadrotor{

	const double pi = 3.14159265;

	// mass / inertia
	const double mQ = 0.546;            // mass of quadcopter [ kg ]
	const double Thxxyy = 2.32e-3;      // moment of inertia around x,y [ kg*m^2 ]
	const double Thzz = 3e-4;          	// moment of inertia around z [ kg*m^2 ]
	const double arm_len = 0.175;       // length of quadcopter arm [ m ]
	const double grav_const = 9.81;		// gravitational constant [ m/s^2 ]

	const double f_hover = mQ*grav_const;

	// Thrust parameters
	const double kF = 6.17092e-8*3600/(2*pi*2*pi);     	// rotor thrust coefficient [ N/rad^2 ]
	const double kM = 1.3167e-9*3600/(2*pi*2*pi);      	// rotor moment coefficient [ Nm/rad^2]
	const double wmax = 7800.0*2*pi/60;  // maximum rotor speed [ rad/s ]
	const double wmin = 1200.0*2*pi/60;  // minimum rotor speed [ rad/s ]
	const double Fsat_min = kF * wmin * wmin;
	const double Fsat_max = kF * wmax * wmax;

	const Eigen::Vector4d kFs(kF, kF, kF, kF);
	const Eigen::Vector4d kMs(kM, kM, kM, kM);

} // namespace quadrotor
} // namespace models
} // namespace ct


#endif /* QUADMODELPARAMETERS_HPP_ */
