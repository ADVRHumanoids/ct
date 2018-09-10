/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/core/core.h>

namespace ct {
namespace models {
namespace HyA {

class HyALinearizedReverse : public ct::core::LinearSystem<12, 6, double>
{
public:
    typedef ct::core::LinearSystem<12, 6, double> Base;

    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::state_matrix_t state_matrix_t;
    typedef typename Base::state_control_matrix_t state_control_matrix_t;

    HyALinearizedReverse(const ct::core::SYSTEM_TYPE& type = ct::core::SYSTEM_TYPE::GENERAL)
        : ct::core::LinearSystem<12, 6>(type)
    {
        initialize();
    }

    HyALinearizedReverse(const HyALinearizedReverse& other) { initialize(); }
    virtual ~HyALinearizedReverse(){};

    virtual HyALinearizedReverse* clone() const override { return new HyALinearizedReverse; }
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const double t = double(0.0)) override;

    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const double t = double(0.0)) override;

private:
    void initialize()
    {
        dFdx_.setZero();
        dFdu_.setZero();
        vX_.fill(0.0);
        vU_.fill(0.0);
    }

    state_matrix_t dFdx_;
    state_control_matrix_t dFdu_;
    std::array<double, 975> vX_;
    std::array<double, 75> vU_;
};
}
}
}
