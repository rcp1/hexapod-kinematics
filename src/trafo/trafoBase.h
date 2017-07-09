#ifndef TRAFOBASE_H
#define TRAFOBASE_H

#include "vector3d.h"

enum trafoStatus
{
	trafoOk,
	trafoErrorZeroDivision,
	trafoErrorSingularPosition,
	trafoErrorInputOutOfBonds,
	trafoErrorParametersOutOfBonds,
	trafoErrorNotImplemented
};

class TrafoBase
{
public:
	TrafoBase();

	virtual ~TrafoBase();

protected:

	/// Forward transformation. Transforms input to output in a "higher" coordinate system.
	/// \param[in]  input       Reference to the input vector
	/// \param[out] output      Reference to the output vector
	/// \return shows success or error
    virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const = 0;

	/// Backward transformation. Transforms input to output in a "lower" coordinate system.
	/// \param[in]  input       Reference to the input vector
	/// \param[out] output      Reference to the output vector
	/// \return shows success or error
	virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const = 0;
};

#endif // TRAFOBASE_H
