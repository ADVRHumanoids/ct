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

#ifndef INCLUDE_CT_RBD_robot_ENDEFFECTOR_H_
#define INCLUDE_CT_RBD_robot_ENDEFFECTOR_H_

#include <ct/rbd/state/JointState.h>
#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class EndEffector {
public:
	typedef Eigen::Matrix<SCALAR,6,NJOINTS> jacobian_t;
	typedef typename JointState<NJOINTS>::Position joint_position_t;

	EndEffector() :
		linkId_(999)
	{};
	virtual ~EndEffector() {};

	EndEffector(const EndEffector& other) :
		linkId_(other.linkId_)
	{}

	//const RigidBodyPose& getPoseOnLink() const { return poseOnLink_; }

	// already in kinematics...
//	RigidBodyPose getPoseOnBase(const joint_position_t& jointState) { throw std::runtime_error("Not implemented"); };


//	jacobian_t getJacobainBase(const joint_position_t& jointState) { throw std::runtime_error("Not implemented"); };

	/**
	 * \brief Return the ID of the link to which the end-effector is rigidly attached to
	 * @return Link ID
	 */
	const size_t& getLinkId() { return linkId_; }

	/**
	 * \brief *DO NOT USE*. Set the link id on which an endeffector is on
	 * @param linkId LinkId to be set
	 */
	void setLinkId(size_t linkId) { linkId_ = linkId;} // we should not have this public

private:
	// id of the link that the endeffector is on
	size_t linkId_;
};

} /* namespace rbd */
} /* namespace ct */

#endif /* INCLUDE_CT_RBD_robot_ENDEFFECTOR_H_ */
