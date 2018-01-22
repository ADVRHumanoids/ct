/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <memory>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include "kinematics/RBDDataMap.h"
#include "ProjectedDynamics.h"
#include "state/JointAcceleration.h"
#include "state/JointState.h"
#include "state/RBDAcceleration.h"
#include "state/RBDState.h"
#include "state/RigidBodyAcceleration.h"
#include "control/SelectionMatrix.h"
#include "KinematicsBase.h"

#define ENABLE_FIX_BASE    \
    template <bool B = FB> \
    typename std::enable_if<!B, void>::type
#define ENABLE_FIX_BASE_IMPL \
    template <bool B>        \
    typename std::enable_if<!B, void>::type
#define ENABLE_FLOAT_BASE  \
    template <bool B = FB> \
    typename std::enable_if<B, void>::type
#define ENABLE_FLOAT_BASE_IMPL \
    template <bool B>          \
    typename std::enable_if<B, void>::type


namespace ct {
namespace rbd {

/**
 * @brief This class is an interface for Dynamic routines of Rigid Body System
 * @tparam NJOINTS  The number of joints
 * @tparam FLOATING_BASE True if the robot has a floating base
 * @tparam SCALAR The scalar type
 */
template <size_t NJOINTS, size_t NLINKS, bool FLOATING_BASE, typename SCALAR=double>
class DynamicsBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const bool FB = FLOATING_BASE;

    // NSTATE either includes the base or is just the joint state
    static const size_t NSTATE = FB * RBDState<NJOINTS, SCALAR>::NSTATE + (1 - FB) * 2 * NJOINTS;

    typedef Eigen::Matrix<SCALAR, NJOINTS, 1> control_vector_t;
    typedef Eigen::Matrix<SCALAR, NSTATE, 1> state_vector_t;
    typedef Eigen::Matrix<SCALAR, 6, 1> Vector6d_t;
    typedef Vector6d_t ForceVector_t;

    typedef RBDState<NJOINTS, SCALAR> RBDState_t;
    typedef RBDAcceleration<NJOINTS, SCALAR> RBDAcceleration_t;
    typedef JointState<NJOINTS, SCALAR> JointState_t;
    typedef JointAcceleration<NJOINTS, SCALAR> JointAcceleration_t;
    typedef tpl::RigidBodyAcceleration<SCALAR> RigidBodyAcceleration_t;

    // currently assumes our input is the joint dimension and we only affect acceleration
    typedef SelectionMatrix<NJOINTS, NSTATE / 2, SCALAR> SelectionMatrix_t;

    typedef ExtLinkForces_t std::array<NLINKS, Vector6d_t>
    typedef RBDDataMap<bool, NEE> EE_in_contact_t;

    typedef KinematicsBase<NJOINTS, SCALAR> Kinematics_t;

    /**
	 * @brief The constructor
	 * @param[in] 	kinematics	The kinematics of the RBD
	 */
    DynamicsBase(typename Kinematics_t::Ptr_t kinematics)
        : S_(FB), kinematics_(kinematics)
    {
    }

    DynamicsBase(const DynamicsBase& other) : S_(other.S_), kinematics_(other.kinematics_->clone()) {}
    virtual ~DynamicsBase(){};


    /**
	 * @brief Compute forward dynamics of a fixed-base RBD system under external
	 * forces
	 * @param[in] 	x		The JointState state
	 * @param[in] 	u		The control vector
	 * @param[in] 	force	The external forces vector
	 * @param[out]	qdd		The joints acceleration
	 */
    virtual ENABLE_FIX_BASE FixBaseForwardDynamics(const JointState_t& x,
        const control_vector_t& u,
        ExtLinkForces_t& force,
        JointAcceleration_t& qdd)
    {
        throw std::runtime_error("FixBaseForwardDynamics not implemented");
        return RigidBodyPoseTpl();
    };

    /**
	 * @brief Compute forward dynamics of a fixed-base RBD system,  NO contact forces
	 * forces
	 * @param[in] 	x	The RBD state
	 * @param[in] 	u	The control vector
	 * @param[out]	qdd	The Joints acceleration
	 */
    virtual ENABLE_FIX_BASE FixBaseForwardDynamics(const JointState_t& x, const control_vector_t& u, JointAcceleration_t& qdd)
    {
        ExtLinkForces_t force(Eigen::Matrix<SCALAR, 6, 1>::Zero());
        FixBaseForwardDynamics(x, u, force, qdd);
    }

    /**
	 * @brief Computes Inverse dynamics of a fixed-base system under external
	 * forces.
	 * @param[in] 	x		the current state of the robot
	 * @param[in] 	qdd		the Joints acceleration
	 * @param[in] 	force	the external forces vector
	 * @param[out]	u		The control vector
	 */
    virtual ENABLE_FIX_BASE FixBaseID(const JointState_t& x,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        control_vector_t& u)
    {
        throw std::runtime_error("FixBaseID not implemented");
    };

    /**
	 * @brief Computes Inverse dynamics of a fixed-base system without external
	 * forces.
	 * @param[in] 	x		the current state of the robot
	 * @param[in] 	qdd		the Joints acceleration
	 * @param[out]	u		The control vector
	 */
    virtual ENABLE_FIX_BASE FixBaseID(const JointState_t& x, const JointAcceleration_t& qdd, control_vector_t& u)
    {
        throw std::runtime_error("FixBaseID not implemented");
    };

    /**
	 * @brief Compute forward dynamics for an floating-base RBD system under external
	 * forces
	 * @param[in] 	x           The RBD state
	 * @param[in] 	u           The control vector
	 * @param[in] 	link_forces The external forces vector
	 * @param[out]	xd          The RBD state derivative
	 */
    virtual ENABLE_FLOAT_BASE FloatingBaseForwardDynamics(const RBDState_t& x,
        const control_vector_t& u,
        const ExtLinkForces_t& link_forces,
        RBDAcceleration_t& xd)
    {
        throw std::runtime_error("FloatingBaseForwardDynamics not implemented");
    };

    /**
	 * @brief Computes Inverse dynamics of a floating-base system under external
	 * forces.
	 * @param[in] 	x		the RBDstate
	 * @param[in] 	qdd		the joints acceleration
	 * @param[in] 	force	the external forces vector
	 * @param[out]	u		The control vector
	 * @param[out]  base_a  The base state derivative
	 */
    virtual ENABLE_FLOAT_BASE FloatingBaseID(const RBDState_t& x,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        control_vector_t& u,
        RigidBodyAcceleration_t& base_a)
    {
         throw std::runtime_error("FloatingBaseID not implemented");
     };

    /**
	 * @brief Computes the inverse dynamics of a floating-base fully-actuated
	 * system
	 * @param[in]	x		The RBDState
	 * @param[in]	base_a	The base acceleration
	 * @param[in]	qdd		The joint acceleration
	 * @param[in]	force	The external forces vector
	 * @param[out]	base_w	The base wrench
	 * @param[out]	u		The control vector
	 */
    virtual ENABLE_FLOAT_BASE FloatingBaseFullyActuatedID(const RBDState_t& x,
        const RigidBodyAcceleration_t& base_ac,
        const JointAcceleration_t& qdd,
        const ExtLinkForces_t& force,
        ForceVector_t& base_w,
        control_vector_t& u)
    {
         throw std::runtime_error("FloatingBaseFullyActuatedID not implemented");
    }



    Kinematics_t& kinematics() { return *kinematics_; }
    const Kinematics_t& kinematics() const { return *kinematics_; }
    typename Kinematics_t::Ptr_t& kinematicsPtr() { return kinematics_; }
    const typename Kinematics_t::Ptr_t& kinematicsPtr() const { return kinematics_; }
    SelectionMatrix_t& S() { return S_; }
    const SelectionMatrix_t& S() const { return S_; }
private:
    SelectionMatrix_t S_;

    typename Kinematics_t::Ptr_t kinematics_; /*!< The RBD kinematics */

protected:
};

#undef ENABLE_FIX_BASE
#undef ENABLE_FLOAT_BASE

}  // namespace rbd
}  // namespace ct
