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

#ifndef INCLUDE_CT_RBD_QUAD_WITH_LOAD_SYSTEM_H_
#define INCLUDE_CT_RBD_QUAD_WITH_LOAD_SYSTEM_H_

#include <ct/core/systems/ControlledSystem.h>
#include <ct/rbd/state/RigidBodyPose.h>

#include <ct/rbd/systems/RBDSystem.h>

namespace ct {
namespace rbd {

template <class RBDDynamics, bool QUAT_INTEGRATION = false>
class QuadWithLoadFDSystem :
		public RBDSystem<RBDDynamics, QUAT_INTEGRATION>,
		public core::ControlledSystem<RBDDynamics::NSTATE+QUAT_INTEGRATION, 4, typename RBDDynamics::SCALAR>
{
public:
	using Dynamics = RBDDynamics;
	using Kinematics = typename RBDDynamics::Kinematics_t;

	typedef typename RBDDynamics::SCALAR SCALAR;

	const static size_t STATE_DIM = RBDDynamics::NSTATE+QUAT_INTEGRATION;
	const static size_t CONTROL_DIM = 4;

	typedef core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;

	QuadWithLoadFDSystem(){};

	QuadWithLoadFDSystem(const QuadWithLoadFDSystem& arg):
		RBDSystem<RBDDynamics, QUAT_INTEGRATION>(arg),
		core::ControlledSystem<RBDDynamics::NSTATE+QUAT_INTEGRATION, 4, typename RBDDynamics::SCALAR>(arg),
		dynamics_(arg.dynamics_)
	{}

	virtual ~QuadWithLoadFDSystem() {};

	virtual RBDDynamics& dynamics() override { return dynamics_; }

	virtual const RBDDynamics& dynamics() const override { return dynamics_; }

	void computeControlledDynamics(
			const core::StateVector<STATE_DIM, SCALAR>& state,
			const core::Time& t,
			const core::ControlVector<CONTROL_DIM, SCALAR>& control,
			core::StateVector<STATE_DIM, SCALAR>& derivative

	) override {
		typename RBDDynamics::RBDState_t x = RBDStateFromVector(state);

		typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());

		mapControlInputsToLinkForces(x, control, linkForces);

		typename RBDDynamics::RBDAcceleration_t xd;
		ct::core::ControlVector<RBDDynamics::NJOINTS> joint_torques = ct::core::ControlVector<RBDDynamics::NJOINTS>::Zero();

		// introduce some light damping into the joint (friction) // todo fixme tune this value
		joint_torques = -0.0005 * x.joints().getVelocities();

		dynamics_.FloatingBaseForwardDynamics(
				x,
				joint_torques,
				linkForces,
				xd);

		derivative = toStateDerivative<QUAT_INTEGRATION>(xd, x);
	}


	void mapControlInputsToLinkForces(
			const typename RBDDynamics::RBDState_t& state,
			const core::ControlVector<CONTROL_DIM, SCALAR>& control,
			typename RBDDynamics::ExtLinkForces_t& linkForces)
	{
		/* u = [overall thrust, mx, my, mz]
		 * we directly map the control inputs into quadrotor body forces
		 * */

		size_t baseLinkId = 0;
		linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)].setZero();
		linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)](5) = control(0);							// the thrust in quadrotor z-direction
		linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(baseLinkId)].template segment<3>(0) =  control.template segment<3>(1); 	// the torques

	}

	typename RBDDynamics::RBDState_t RBDStateFromVector(const core::StateVector<STATE_DIM, SCALAR>& state)
	{
		return RBDStateFromVectorImpl<QUAT_INTEGRATION>(state);
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state, typename std::enable_if<T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(RigidBodyPose::QUAT);
		x.fromStateVectorQuaternion(state);
		return x;
	}

	template <bool T>
	typename RBDDynamics::RBDState_t RBDStateFromVectorImpl(const core::StateVector<STATE_DIM, SCALAR>& state, typename std::enable_if<!T, bool>::type = true)
	{
		typename RBDDynamics::RBDState_t x(RigidBodyPose::EULER);
		x.fromStateVectorEulerXyz(state);
		return x;
	}

	template <bool T>
	core::StateVector<STATE_DIM> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<T, bool>::type = true) {
		return acceleration.toStateUpdateVectorQuaternion(state);
	}

	template <bool T>
	core::StateVector<STATE_DIM> toStateDerivative(
			const typename RBDDynamics::RBDAcceleration_t& acceleration,
			const typename RBDDynamics::RBDState_t& state,
			typename std::enable_if<!T, bool>::type = true) {
		return acceleration.toStateUpdateVectorEulerXyz(state);
	}


	virtual QuadWithLoadFDSystem<RBDDynamics, QUAT_INTEGRATION>* clone() const override { return new QuadWithLoadFDSystem(*this);}

private:
	RBDDynamics dynamics_;

};

}
}


#endif /* INCLUDE_CT_RBD_QUAD_WITH_LOAD_SYSTEM_H_ */
