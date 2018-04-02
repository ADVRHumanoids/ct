/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DEBUG


namespace ct {
namespace optcon {
namespace example {


TEST(CostFunctionParametrizedTest, CostFunctionParametrizedJITTest)
{
    const size_t state_dim = 2;
    const size_t control_dim = 1;
    const size_t n_param = 1;

    CostFunctionAnalytical<state_dim, control_dim, ct::core::ADCGScalar> costFunction;
 
    // create a linear cost function term of L(x,u) = a^T x + b^T u + c
    // we assume a and c are constant
    Eigen::StateVector<state_dim, ct::core::ADCGScalar> a;
    a << 1.0, 2.0;
    ct::core::ADCGScalar c(0.0);

    
    // we assume b is a parameter, this parameter is just used to initialize the class but overwritten later
    Eigen::ControlVector<control_dim, ct::core::ADCGScalar> b;
    b << 0.5;

    // create the term
    std::shared_ptr<TermLinear<state_dim, control_dim, ct::core::ADCGScalar, ct::core::ADCGScalar>> termLinear(
        new TermLinear<state_dim, control_dim, ct::core::ADCGScalar, ct::core::ADCGScalar>(a, b, c));

    costFunction.addIntermediateTerm(termLinear);


    // auto-diff state size (consists of state, control and parameters)
    const size_t autodiff_dim = state_dim+control_dim+n_param;

    // create a lambda that takes an ad vector containing state, control and parameter. In this function set the parameter and then evaluate the cost.
    auto setParamsAndEvaluate = Eigen::Matrix<ct::core::ADCGScalar, 1, 1> [costfunction&, termLinear&](Eigen::Matrix<ct::core::ADCGScalar, autodiff_dim, 1> ad_vector)
	{
          // we assume ad_vector contains [x^T, u^T, b^T]^T
	  // extract the parameter, control and state (verbose method, can be shortened)
	  Eigen::ControlVector<control_dim, ct::core::ADCGScalar> b = ad_vector.template tail<n_param>(); // we assume parameter is last, this is an arbitrary choice
          Eigen::StateVector<state_dim, ct::core::ADCGScalar> x = ad_vector.template head<state_dim>(ad_vector);
          Eigen::ControlVector<control_dim, ct::core::ADCGScalar> u = ad_vector.template segment<control_dim>(state_dim);

          // set the parameter in the term
          termLinear->b() = b;

          // evaluate the cost function (using x and u, t = 0.0)
          return costFunction.evaluate(x, u, 0.0);
        }

    // input are state, control and parameters, output is a scalar (cost)
    ct::core::DerivativesCppadJIT<state_dim+control_dim+n_param, 1> derivatives(setParamsAndEvaluate);

    derivatives.compileJIT();

    // test
    for (size_t i=0: i<nTests; i++)
    {
      Eigen::VectorXd ad_vec;
      Eigen::StateVector<state_dim> x_test; x_test.setRandom();
      Eigen::ControlVector<control_dim> u_test; u_test.setRandom();
      double c_test = randn();
      
      ad_test << x_test, u_test, c_test;
      double eval = derivatives.forwardZero(ad_test);
      eval_test = a.template cast<double>().transpose() * x_test + b.template cast<double>() * u_test + c_test;

      ASSERT_NEAR(eval, eval_test, 1e-12)

      auto jac = derivatives.jacobian(ad_test);
      // partial derivative of cost with respect to input should be c
      ASSERT_NEAR(jac(state_dim, 1), c_test, 1e-12)
    }

}



}  // namespace example
}  // namespace optcon
}  // namespace ct
