/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_CORE_CORE_H_
#define INCLUDE_CT_CORE_CORE_H_

#include <iosfwd>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include <cppad/cg.hpp>

#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/example/eigen_mat_inv.hpp>
#include "internal/autodiff/CppadParallel.h"

// Include file for convenience
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

// Declarations
#include "Common"
#include "Types"
#include "Control"
#include "Systems"
#include "Integration"
#include "Geometry"
#include "Internal"
#include "Math"
#include "Simulation"
#include "Switching"

#include "templateDir.h"

#ifdef PLOTTING_ENABLED
#include "plot/plot.h"
#endif


// Implementations
#include "Common-impl"
#include "Types-impl"
#include "Control-impl"
#include "Systems-impl"
#include "Integration-impl"
#include "Internal-impl"
#include "Math-impl"
#include "Geometry-impl"
#include "Simulation-impl"

// keep standard header guard (easy debugging)
// header guard is identical to the one in core-prespec.h
#endif  // INCLUDE_CT_CORE_CORE_H_
