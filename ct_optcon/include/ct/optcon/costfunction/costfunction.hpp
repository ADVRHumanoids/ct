/*!
 * a convenience include file collecting all class declarations related to cost functions
 */

#pragma once

// time activations
#include "term/TermBase.hpp"
#include "term/TermLinear.hpp"
#include "term/TermMixed.hpp"
#include "term/TermQuadratic.hpp"
#include "term/TermQuadMult.hpp"
#include "term/TermQuadTracking.hpp"

#ifdef CPPAD_ENABLED
#include "term/TermStateBarrier.hpp"
#endif

// costfunctions
#include "CostFunction.hpp"
#include "CostFunctionQuadratic.hpp"
#include "CostFunctionAnalytical.hpp"
#include "CostFunctionQuadraticSimple.hpp"

#ifdef CPPAD_ENABLED
#include "CostFunctionAD.hpp"
#endif
