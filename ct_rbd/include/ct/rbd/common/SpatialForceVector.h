/***********************************************************************************
Copyright (c) 2017, ETH Zurich, Google Inc. All rights reserved.
Authors: Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. 

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#ifndef INCLUDE_CT_RBD_ROBOT_KINEMATICS_EEFORCE_H_
#define INCLUDE_CT_RBD_ROBOT_KINEMATICS_EEFORCE_H_

namespace ct {
namespace rbd {

/*!
 * \brief A spatial force vector
 * This vector contains a torque (angular) in the upper three rows
 * and a linear force in the lower three rows
 */
template <typename SCALAR = double>
class SpatialForceVector : public Eigen::Matrix<SCALAR, 6, 1>
{
public:
	typedef Eigen::Matrix<SCALAR, 6, 1> spatial_force_vector_t; //!< special vector type
	typedef Eigen::VectorBlock<spatial_force_vector_t, 3> ForceTorqueBlock; //!< 3D force or torque block inside of the spacial vector
	typedef Eigen::VectorBlock<const spatial_force_vector_t, 3> ForceTorqueBlockConst; //!< const 3D force or torque block inside of the spacial vector

	/*!
	 * \brief Default constructor
	 */
	SpatialForceVector()
	{
	}

	/*!
	 * \brief Copy constructor
	 */
	SpatialForceVector(const spatial_force_vector_t& vector6) :
		spatial_force_vector_t(vector6)
	{
	}

	/*!
	 * \brief Returns the Eigen implementation
	 */
	spatial_force_vector_t& toImplementation() { return *this; }

	/*!
	 * \brief Returns the Eigen implementation
	 */
	const spatial_force_vector_t& toImplementation() const { return *this; }

	/*!
	 * \brief Sets all entries to zero
	 */
	SpatialForceVector& setZero() {
		// for auto diff compatability
		SCALAR zero(0.0);
		this->setConstant(zero);
		return *this;
	}

	/*!
	 * \brief Comparison operator (exact, subject to floating point rounding)
	 */
	inline bool operator == (const spatial_force_vector_t& rhs) const
	{
		return Base::operator==(rhs);
	}

	/*!
	 * \brief Comparison operator (exact, subject to floating point rounding)
	 */
	inline bool operator != (const spatial_force_vector_t& rhs) const
	{
		return Base::operator!=(rhs);
	}

	/*!
	 * \brief Sums two spatial forces component-wise
	 */
	inline spatial_force_vector_t operator + (const spatial_force_vector_t& rhs) const
	{
		return spatial_force_vector_t(Base::operator+(rhs));
	}
	/*!
	 * \brief Substracts two spatial forces component-wise
	 */
	inline spatial_force_vector_t operator - (const spatial_force_vector_t& rhs) const
	{
		return spatial_force_vector_t(Base::operator-(rhs));
	}

	/*!
	 * \brief Scales a spatial forces component-wise
	 */
	inline spatial_force_vector_t operator * (const SCALAR& scalar) const
	{
		return spatial_force_vector_t(Base::operator*(scalar));
	}

	/*!
	 * \brief Divides a spatial forces component-wise
	 */
	inline spatial_force_vector_t operator / (const SCALAR& scalar) const
	{
		return spatial_force_vector_t(Base::operator/(scalar));
	}

	/*!
	 * \brief Get the torque block (upper 3D block)
	 */
	ForceTorqueBlock torque() { return this->template head<3>(); }

	/*!
	 * \brief Get the torque block (upper 3D block)
	 */
	const ForceTorqueBlockConst torque() const { return this->template head<3>(); }

	/*!
	 * \brief Get the force block (lower 3D block)
	 */
	ForceTorqueBlock force() { return this->template tail<3>(); }
	/*!
	 * \brief Get the force block (lower 3D block)
	 */
	const ForceTorqueBlockConst force() const { return this->template tail<3>(); }

private:
	typedef Eigen::Matrix<SCALAR, 6, 1> Base;
};

}
}



#endif /* INCLUDE_CT_RBD_ROBOT_KINEMATICS_EEFORCE_H_ */
