/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
 **********************************************************************************************************************/

#include <ct/optcon/optcon-prespec.h>
#include "LinearSystemTest.h"

/*!
 * This runs the GNMS unit test.
 * \note for a more straight-forward implementation example, visit the tutorial.
 * \example GNMSCTest.cpp
 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    //	ct::optcon::example::singleCore();
    //	ct::optcon::example::multiCore();

    return 1;
}
