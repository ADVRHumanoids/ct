/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace models {
namespace DoubleInvertedPendulum {

const std::vector<std::string>& urdfJointNames()
{
    static std::vector<std::string> urdfJointNames{"Joint1", "Joint2"};

    return urdfJointNames;
}
}
}
}
