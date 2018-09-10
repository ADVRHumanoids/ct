/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/state/JointState.h>
#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class InverseKinematicsBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using JointPosition_t = typename JointState<NJOINTS, SCALAR>::Position;
    using JointPositionsVector_t = std::vector<JointPosition_t, Eigen::aligned_allocator<JointPosition_t>>;
    using RigidBodyPoseTpl = tpl::RigidBodyPose<SCALAR>;

    //! default constructor
    InverseKinematicsBase() = default;

    //! default destructor
    virtual ~InverseKinematicsBase() = default;

    /*!
     * @brief compute inverse kinematics for an end-effector w.r.t robot base
     * @param ikSolutions vector of all solutions to the inverse kinematics problem
     * @param eeBasePose end-effector pose in base coordinates
     * @param freeJoints vector of indices of the free joints
     * @return true if solution found, false otherwise
     */
    virtual bool computeInverseKinematics(JointPositionsVector_t& ikSolutions,
        const RigidBodyPoseTpl& eeBasePose,
        const std::vector<size_t>& freeJoints) const = 0;

    /*!
     * @brief compute inverse kinematics for an end-effector
     * @param ikSolutions vector of all solutions to the inverse kinematics problem
     * @param eeWorldPose end-effector pose in world coordinates
     * @param baseWorldPose base pose in world coordinates
     * @param freeJoints vector of indices of the free joints
     * @return true if solution found, false otherwise
     */
    virtual bool computeInverseKinematics(JointPositionsVector_t& ikSolutions,
        const RigidBodyPoseTpl& eeWorldPose,
        const RigidBodyPoseTpl& baseWorldPose,
        const std::vector<size_t>& freeJoints) const = 0;


    /*!
     * @brief compute inverse kinematics for an end-effector
     * @param ikSolution solution to the inverse kinematics problem which is closest to parameter 'queryJointPositions'
     * @param eeWorldPose end-effector pose in world coordinates
     * @param baseWorldPose base pose in world coordinates
     * @param freeJoints vector of indices of the free joints
     * @return true if solution found, false otherwise
     */
    virtual bool computeInverseKinematicsCloseTo(JointPosition_t& ikSolution,
        const RigidBodyPoseTpl& eeWorldPose,
        const RigidBodyPoseTpl& baseWorldPose,
        const JointPosition_t& queryJointPositions,
        const std::vector<size_t>& freeJoints = std::vector<size_t>()) const
    {
        // compute all inverse kinematics solutions
        JointPositionsVector_t solutions;
        bool hasSolution = computeInverseKinematics(solutions, eeWorldPose, baseWorldPose, freeJoints);

        if (!hasSolution)
            return false;  // no IK solution was found

        // loop through all solution candidates and return the one which is closest to the "queryJointPositions"
        ikSolution = solutions[0];
        double minNorm = (ikSolution - queryJointPositions).norm();

        for (int i = 1; i < solutions.size(); ++i)
        {
            if ((solutions[i] - queryJointPositions).norm() < minNorm)
            {
                minNorm = (solutions[i] - queryJointPositions).norm();
                ikSolution = solutions[i];
            }
        }
        return true;
    }

    /*!
     * @brief compute inverse kinematics for an end-effector
     * @param ikSolution solution to the inverse kinematics problem which is closest to parameter 'queryJointPositions'
     * @param eeBasePose end-effector pose in base coordinates
     * @param freeJoints vector of indices of the free joints
     * @return true if solution found, false otherwise
     */
    virtual bool computeInverseKinematicsCloseTo(JointPosition_t& ikSolution,
        const RigidBodyPoseTpl& eeBasePose,
        const JointPosition_t& queryJointPositions,
        const std::vector<size_t>& freeJoints = std::vector<size_t>()) const
    {
        // set base world pose as identity, herewith eeBasePose = eeWorldPose
        RigidBodyPoseTpl identityWorldPose;
        identityWorldPose.setIdentity();
        return computeInverseKinematicsCloseTo(
            ikSolution, eeBasePose, identityWorldPose, queryJointPositions, freeJoints);
    }
};

} /* namespace rbd */
} /* namespace ct */
