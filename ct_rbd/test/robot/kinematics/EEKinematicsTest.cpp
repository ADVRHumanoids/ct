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

#include <ct/rbd/rbd.h>
#include <memory>

#include <gtest/gtest.h>

#include <ct/rbd/state/RBDState.h>

#include "../../models/testhyq/RobCoGenTestHyQ.h"

using namespace ct::rbd;

typedef RBDState<12> RBDStateHyQ;

const size_t nFeet = 4;

// Test influence of base angular velocity on feet
TEST(EEKinematicsTest, testFootVelocityBaseAngularVelocity)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;

	// straight pose, all legs streched
	state.setZero();

	// rotate around x
	state.baseLocalAngularVelocity()(0) = 1.3;

	for (size_t i=0; i<nFeet; i++)
	{
		kindr::Velocity3D eeVelW;
		kindr::Velocity3D eeVelB;

		eeVelW = kinematics.getEEVelocityInWorld(i, state);
		eeVelB = kinematics.getEEVelocityInWorld(i, state);

		// Since world and base are aligned, both velocities should be the same
		ASSERT_TRUE(eeVelW.toImplementation().isApprox(eeVelB.toImplementation()));

		// x component should be zero
		ASSERT_NEAR(eeVelW(0), 0.0, 1e-6);

		// y component should be greater zero
		ASSERT_GT(eeVelW(1), 0.0);
	}
}


// Test influence of base angular velocity on feet
TEST(EEKinematicsTest, testFootVelocityBaseZRotation)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;

	// random configuration
	state.setRandom();

	// straight orientation, no joint velocity
	state.basePose().setFromEulerAnglesXyz(Eigen::Vector3d(0.0, 0.0, 0.0));
	state.jointVelocities().setZero();

	// rotate around z only
	state.baseLocalAngularVelocity()(0) = 0.0;
	state.baseLocalAngularVelocity()(1) = 0.0;
	state.baseLocalAngularVelocity()(2) = 1.3;

	const size_t nFeet = 4;
	for (size_t i=0; i<nFeet; i++)
	{
		kindr::Velocity3D eeVelW;
		kindr::Velocity3D eeVelB;
		eeVelW = kinematics.getEEVelocityInWorld(i, state);
		eeVelB = kinematics.getEEVelocityInWorld(i, state);

		// Since world and base are aligned, both velocities should be the same
		ASSERT_TRUE(eeVelW.toImplementation().isApprox(eeVelB.toImplementation()));

		// z component should be linear velocity in z
		ASSERT_NEAR(eeVelW(2), state.baseLinearVelocity()(2), 1e-6);
	}
}


// Test influence of base linear velocity on feet
TEST(EEKinematicsTest, testFootVelocityBaseLinearVelocity)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;


	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		// straight orientation, no rotational velocity
		state.basePose().setFromEulerAnglesXyz(Eigen::Vector3d(0.0, 0.0, 0.0));
		state.baseLocalAngularVelocity().setZero();
		state.jointVelocities().setZero();

		const size_t nFeet = 4;
		for (size_t i=0; i<nFeet; i++)
		{
			kindr::Velocity3D eeVelW;
			kindr::Velocity3D eeVelB;
			eeVelW = kinematics.getEEVelocityInWorld(i, state);
			eeVelB = kinematics.getEEVelocityInWorld(i, state);

			// Since world and base are aligned, both velocities should be the same
			ASSERT_TRUE(eeVelW.toImplementation().isApprox(eeVelB.toImplementation()));

			// Velocity of feet should be equal to velocity of base
			ASSERT_TRUE(eeVelW.toImplementation().isApprox(state.baseLinearVelocity().toImplementation()));
		}
	}
}


// Test foot positions in base
TEST(EEKinematicsTest, testFootPositionVaryingBase)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;


	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		state.jointPositions().setZero();

		const size_t nFeet = 4;
		std::array<kindr::Position3D, nFeet> B_eePos;
		for (size_t i=0; i<nFeet; i++)
		{
			 B_eePos[i] = kinematics.getEEPositionInBase(i, state.jointPositions());

			 // all legs should have same height
			if (i>0)
				ASSERT_NEAR(B_eePos[0](2), B_eePos[i](2), 1e-6);

			// legs should be below belly
			ASSERT_LT(B_eePos[0](2), -0.5);
			ASSERT_GT(B_eePos[0](2), -1.5);
		}
	}
}


// Test influence of base linear velocity on feet
TEST(EEKinematicsTest, testFootPositionStraightBase)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;


	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		// straight orientation, no rotational velocity
		state.basePose().setFromEulerAnglesXyz(Eigen::Vector3d(0.0, 0.0, 0.0));

		state.jointPositions().setZero();

		const size_t nFeet = 4;
		std::array<kindr::Position3D, nFeet> W_eePos;
		for (size_t i=0; i<nFeet; i++)
		{
			W_eePos[i] = kinematics.getEEPositionInWorld(i, state.basePose(), state.jointPositions());

			 // all legs should have same height
			if (i>0)
				ASSERT_NEAR(W_eePos[0](2), W_eePos[i](2), 1e-6);

			// legs should be below belly
			ASSERT_LT(W_eePos[0](2), state.basePose().position()(2)-0.5);
			ASSERT_GT(W_eePos[0](2), state.basePose().position()(2)-1.5);


		}
	}
}



// Test influence of base linear velocity on feet
TEST(EEKinematicsTest, forceMappingTest)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;


	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		// straight orientation, no rotational velocity
		state.basePose().setFromEulerAnglesXyz(Eigen::Vector3d(0.0, 0.0, 0.0));

		state.jointPositions().setZero();

		const size_t nFeet = 4;

		std::array<TestHyQ::Kinematics::EEForce, nFeet> eeForcesW;


		for (size_t i=0; i<nFeet; i++)
		{
			// only force in z
			eeForcesW[i].setZero();
			eeForcesW[i].segment<1>(5).setRandom();

			TestHyQ::Kinematics::EEForce eeForceLink;
			eeForceLink.setZero();

			eeForceLink = kinematics.mapForceFromWorldToLink(eeForcesW[i],
					state.basePose(),
					state.jointPositions(),
					i);

			// we only expect forces in negative x, no torques
			for (size_t j=0; j<6; j++)
			{
				if (j!=3)
					ASSERT_NEAR(eeForceLink(j), 0.0, 1e-6);
			}

			ASSERT_NEAR(eeForceLink(3), -eeForcesW[i](5), 1e-6);
		}
	}
}




TEST(EEKinematicsTest, forceMagnitudeTest)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;

	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		const size_t nFeet = 4;
		std::array<TestHyQ::Kinematics::EEForce, nFeet> eeForcesW;

		for (size_t i=0; i<nFeet; i++)
		{
			eeForcesW[i].setRandom();

			TestHyQ::Kinematics::EEForce forceLink;

			forceLink = kinematics.mapForceFromWorldToLink(eeForcesW[i],
								state.basePose(),
								state.jointPositions(),
								i);

			// we expect the magnitude not to change
			ASSERT_NEAR(forceLink.bottomRows<3>().norm(), eeForcesW[i].bottomRows<3>().norm(), 1e-6);
		}
	}
}



// Test influence of base linear velocity on feet
TEST(EEKinematicsTest, torqueMappingTest)
{
	RBDStateHyQ state;
	TestHyQ::Kinematics kinematics;


	const size_t nTests = 100;

	for(size_t t=0; t<nTests; t++)
	{
		// random configuration
		state.setRandom();

		// straight orientation, no rotational velocity
		state.basePose().setFromEulerAnglesXyz(Eigen::Vector3d(0.0, 0.0, 0.0));

		state.jointPositions().setZero();

		const size_t nFeet = 4;

		std::array<TestHyQ::Kinematics::EEForce, nFeet> eeForcesW;


		for (size_t i=0; i<nFeet; i++)
		{
			state.jointPositions()(3*i+2) = M_PI/2.0; // knees bent by 90°

			// only force in z
			eeForcesW[i].setZero();
			eeForcesW[i].segment<1>(5).setRandom();

			TestHyQ::Kinematics::EEForce forceLink;

			forceLink = kinematics.mapForceFromWorldToLink(eeForcesW[i],
								state.basePose(),
								state.jointPositions(),
								i);

			// we only expect forces in negative y
			ASSERT_NEAR(forceLink(3), 0.0, 1e-6);
			ASSERT_NEAR(forceLink(4), eeForcesW[i](5),1e-6);
			ASSERT_NEAR(forceLink(5), 0.0, 1e-6);

			// we expect torques only around z
			for (size_t j=0; j<2; j++)
			{
				ASSERT_NEAR(forceLink(j), 0.0, 1e-6);
			}

			double torqueAbs = std::abs(forceLink(2));
			ASSERT_NEAR(torqueAbs, forceLink.bottomRows<3>().norm()*0.33, 1e-6);

			if (i<2 && (i-1)%3 != 0)
			{
				ASSERT_NEAR(forceLink.bottomRows<3>().norm(), 0.0, 1e-12);
			}

		}
	}
}





int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
