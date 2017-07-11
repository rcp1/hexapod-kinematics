#include "rotate.h"
#include "vector3d.h"
#include "orientation3d.h"
#include <math.h>

namespace math
{

Vector3d rotateX(const Vector3d& input, float angle)
{
	Vector3d result;

	result.x =  input.x;
	result.y =  cosf(angle) * input.y - sinf(angle) * input.z;
	result.z =  sinf(angle) * input.y + cosf(angle) * input.z;

	return result;
}

Vector3d rotateY(const Vector3d& input, float angle)
{
	Vector3d result;

	result.x = cosf(angle) * input.x + sinf(angle) * input.z;
	result.y = input.y;
	result.z = -sinf(angle) * input.x + cosf(angle) * input.z;

	return result;
}

Vector3d rotateZ(const Vector3d& input, float angle)
{
	Vector3d result;

	result.x = cosf(angle) * input.x - sinf(angle) * input.y;
	result.y = sinf(angle) * input.x + cosf(angle) * input.y;
	result.z = input.z;

	return result;
}

Vector3d rotateToOrientation(const Vector3d& input, const Orientation3d& orientation)
{
	Vector3d result(input);

	// Yaw
	result = math::rotateZ(result, orientation.psi);
	// Pitch
	result = math::rotateY(result, orientation.theta);
	// Roll
	result = math::rotateX(result, orientation.phi);

	return result;
}

}
