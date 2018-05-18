/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

#include <ct/rbd/rbd.h>

#include "generated/declarations.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/jacobians.h"
#include "generated/jsim.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
#include "generated/traits.h"


#define ROBCOGEN_NS testirb4600  // defines the NS of the robot in robcogen, e.g. iit::<ROBCOGEN_NS>
#define TARGET_NS \
    TestIrb4600  // defines the NS where all robot definitions go. Here ct::rbd::TestIrb4600. This defines ct::rbd::TestIrb4600::Dynamics etc.


// define the links
#define CT_BASE fr_link0
#define CT_L0 fr_link1
#define CT_L1 fr_link2
#define CT_L2 fr_link3
#define CT_L3 fr_link4
#define CT_L4 fr_link5
#define CT_L5 fr_link6

// define single end effector (could also be multiple)
#define CT_N_EE 1;
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 5
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 5

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
