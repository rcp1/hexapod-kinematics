#ifndef TRAFOCOORDREFLECTION_H
#define TRAFOCOORDREFLECTION_H

#include "trafoBase.h"

class TrafoCoordReflection : public TrafoBase
{
public:
	TrafoCoordReflection(uint8_t axis = 'x');

	virtual ~TrafoCoordReflection();

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

	void setAxis(const uint8_t& axis);

protected:
	Vector3d reflect(const Vector3d& input) const;

	uint8_t m_axis;
};

#endif // TRAFOCOORDREFLECTION_H
