/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_CORE_CORE_H_
#define INCLUDE_CT_CORE_CORE_H_

#include <iosfwd>
#include <vector>
#include <cstdlib>

#ifdef CPPAD_ENABLED
#include <cppad/cg.hpp>

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/example/eigen_mat_inv.hpp>
#include "internal/autodiff/CppadParallel.h"
#endif

// Include file for convenience
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>


#include "Internal"
#include "Common"
#include "Types"
#include "Control"
#include "Systems"
#include "Integration"
#include "Geometry"
#include "Math"

#include "templateDir.h"

#ifdef PLOTTING_ENABLED
#include "plot/plot.h"
#endif

/*!
 * \warning{do not include implementation files in core-prespec.h}
 */


// keep standard header guard (easy debugging)
// header guard is identical to the one in core.h
#endif  // INCLUDE_CT_CORE_CORE_H_
