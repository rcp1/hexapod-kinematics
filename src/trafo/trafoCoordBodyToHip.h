#ifndef TRAFOBODYTOHIP_H
#define TRAFOBODYTOHIP_H

#include "trafoBase.h"
#include "defines.h"
#include "trafoCoordRotation.h"
#include "trafoCoordReflection.h"

class TrafoCoordBodyToHip : public TrafoBase
{
public:
	TrafoCoordBodyToHip();

	virtual ~TrafoCoordBodyToHip();

	/// Forward transformation. Transforms input to output in a "higher" coordinate system.
	/// \param[in]  input       Reference to the input vector
	/// \param[out] output      Reference to the output vector
	/// \return shows success or error
	virtual trafoStatus forward(const Vector3d& input, Vector3d& output, uint8_t legIndex) const;

	virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const;

	/// Backward transformation. Transforms input to output in a "lower" coordinate system.
	/// \param[in]  input       Pointer to the input vector
	/// \param[out] output      Pointer to the output vector
	/// \return shows success or error
	virtual trafoStatus backward(const Vector3d& input, Vector3d& output, uint8_t legIndex) const;

	virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const;

protected:
	float getPhi0(uint8_t legIndex) const;

	Vector3d createVector(uint8_t legIndex) const;

	TrafoCoordReflection m_trafoCoordReflectionX;
};

inline
trafoStatus TrafoCoordBodyToHip::forward(const Vector3d& input, Vector3d& output) const
{
	return forward(input, output, msrh01::legIndex::mr);
};

inline
trafoStatus TrafoCoordBodyToHip::backward(const Vector3d& input, Vector3d& output) const
{
	return backward(input, output, msrh01::legIndex::mr);
};

#endif // TRAFOBODYTOHIP_H
