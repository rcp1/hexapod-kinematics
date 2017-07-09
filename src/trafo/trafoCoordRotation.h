#ifndef TRAFOROTATION_H
#define TRAFOROTATION_H

#include "trafoBase.h"
#include <stdint.h>

class TrafoRotation : public TrafoBase
{
public:
	TrafoRotation(uint8_t axis = 'x', float angleForward = 0.0f);

	virtual ~TrafoRotation();

	/// Forward transformation. Transforms input to output in a "higher" coordinate system.
	/// \param[in]  input       Reference to the input vector
	/// \param[out] output      Reference to the output vector
	/// \return shows success or error
	virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const;

	/// Backward transformation. Transforms input to output in a "lower" coordinate system.
	/// \param[in]  input       Reference to the input vector
	/// \param[out] output      Reference to the output vector
	/// \return shows success or error
	virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const;

	void setAxisAndAngle(const uint8_t& axis, const float& angleForward);

protected:
	trafoStatus rotate(const Vector3d& input, Vector3d& output, const float& angle) const;

	uint8_t m_axis;

	float m_angleForward;
};


#endif // TRAFOROTATION_H
