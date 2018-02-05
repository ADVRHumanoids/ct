/*!
 * a convenience include file collecting all class implementations related to cost functions
 */

#pragma once

// terms
#include "term/TermBase-impl.hpp"
#include "term/TermLinear-impl.hpp"
#include "term/TermMixed-impl.hpp"
#include "term/TermQuadratic-impl.hpp"
#include "term/TermQuadMult-impl.hpp"
#include "term/TermQuadTracking-impl.hpp"

#ifdef CPPAD_ENABLED
#include "term/TermStateBarrier-impl.hpp"
#endif

// costfunctions
#include "CostFunction-impl.hpp"

#ifdef CPPAD_ENABLED
#include "CostFunctionAD-impl.hpp"
#endif

#include "CostFunctionAnalytical-impl.hpp"
#include "CostFunctionQuadratic-impl.hpp"
#include "CostFunctionQuadraticSimple-impl.hpp"
