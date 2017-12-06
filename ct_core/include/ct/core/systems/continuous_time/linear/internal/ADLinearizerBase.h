/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <ct/core/internal/autodiff/SparsityPattern.h>

namespace ct {
namespace core {
namespace internal {

//! Base class for Auto-Diff and Auto-Diff Codegen linearization
/*!
 * This class contains shared code between Auto-Diff and Auto-Diff Codegen linearization.
 *
 * \tparam STATE_DIM dimension of state vector
 * \tparam CONTROL_DIM dimension of control vector
 * \tparam SCALAR scalar type
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR>
class ADLinearizerBase : public LinearSystem<STATE_DIM, CONTROL_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert((std::is_same<SCALAR, CppAD::AD<double>>::value) ||
                      (std::is_same<SCALAR, CppAD::AD<CppAD::cg::CG<double>>>::value),
        "SCALAR template parameter in ADLinearizerBase should either be of CppAD::AD<double> or "
        "CppAD::AD<CppAD::cg::double> type");

    typedef StateVector<STATE_DIM> state_vector_t;
    typedef ControlVector<CONTROL_DIM> control_vector_t;

    typedef StateVector<STATE_DIM, SCALAR> state_vector_ad_t;
    typedef ControlVector<CONTROL_DIM, SCALAR> control_vector_ad_t;

    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> state_control_matrix_t;

    //! default constructor
    /*!
	 * @param nonlinearSystem non-linear system to linearize
	 */
    ADLinearizerBase(std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> nonlinearSystem)
        : LinearSystem<STATE_DIM, CONTROL_DIM>(nonlinearSystem->getType()), nonlinearSystem_(nonlinearSystem)
    {
        initialize();
    }

    //! copy constructor
    ADLinearizerBase(const ADLinearizerBase& arg)
        : LinearSystem<STATE_DIM, CONTROL_DIM>(arg), nonlinearSystem_(arg.nonlinearSystem_->clone())
    {
        setupSparsityA();
        setupSparsityB();
        f_ = arg.f_;
    }

    //! destructor
    virtual ~ADLinearizerBase() {}
protected:
    const size_t A_entries = STATE_DIM * STATE_DIM;    //!< number of entries in the state Jacobian
    const size_t B_entries = STATE_DIM * CONTROL_DIM;  //!< number of entries in the input Jacobian
    const size_t FullJac_entries =
        (STATE_DIM + CONTROL_DIM) * STATE_DIM;  //!< number of entries in the stacked Jacobian

    //! initialize all utilities
    /*!
	 * Records the model function and sets up the sparsity patterns for the Jacobians.
	 */
    void initialize()
    {
        recordTerms();
        setupSparsityA();
        setupSparsityB();
    }

    //! record the model
    void recordTerms()
    {
        // input vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> x(STATE_DIM + CONTROL_DIM);

        x.setZero();

        // declare x as independent
        CppAD::Independent(x);

        // create fixed size types since CT uses fixed size types
        state_vector_ad_t xFixed = x.template head<STATE_DIM>();
        control_vector_ad_t uFixed = x.template tail<CONTROL_DIM>();

        state_vector_ad_t dxFixed;

        nonlinearSystem_->computeControlledDynamics(xFixed, SCALAR(0.0), uFixed, dxFixed);

        // output vector, needs to be dynamic size
        Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> dx(STATE_DIM);
        dx = dxFixed;

        // store operation sequence in f: x -> dx and stop recording
        CppAD::ADFun<typename SCALAR::value_type> f(x, dx);

        f.optimize();

        f_ = f;
    }

    //! setup the sparsity of the state Jacobian
    void setupSparsityA()
    {
        // the derivative is a STATE_DIM*STATE_DIM Matrix:
        // dF/dx = [ A, B ]^T
        Eigen::Matrix<bool, STATE_DIM + CONTROL_DIM, STATE_DIM> sparsity;
        sparsity.setZero();
        sparsity.template topRows<STATE_DIM>().setOnes();

        sparsityA_.initPattern(sparsity);
        sparsityA_.clearWork();
    }

    //! setup the sparsity of the input Jacobian
    void setupSparsityB()
    {
        // the derivative is a STATE_DIM*CONTROL_DIM Matrix:
        // dF/dx = [ A, B ]^T
        Eigen::Matrix<bool, STATE_DIM + CONTROL_DIM, STATE_DIM> sparsity;
        sparsity.setZero();
        sparsity.template bottomRows<CONTROL_DIM>().setOnes();

        sparsityB_.initPattern(sparsity);
        sparsityB_.clearWork();
    }

    std::shared_ptr<ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>>
        nonlinearSystem_;  //!< instance of the non-linear system

    CppAD::ADFun<typename SCALAR::value_type> f_;  //!< Auto-Diff function

    SparsityPattern sparsityA_;  //!< sparsity pattern of the state Jacobian
    SparsityPattern sparsityB_;  //!< sparsity pattern of the input Jacobian
};
}
}
}