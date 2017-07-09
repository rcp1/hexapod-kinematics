#include "trafoCoordReflection.h"

TrafoCoordReflection::TrafoCoordReflection(uint8_t axis) : m_axis(axis)
{
}


TrafoCoordReflection::~TrafoCoordReflection()
{
}

trafoStatus TrafoCoordReflection::forward(const Vector3d & input, Vector3d & output) const
{
	trafoStatus result = trafoOk;

	output = reflect(input);

	return result;
}

trafoStatus TrafoCoordReflection::backward(const Vector3d & input, Vector3d & output) const
{
	trafoStatus result = trafoOk;

	output = reflect(input);

	return result;
}

Vector3d TrafoCoordReflection::reflect(const Vector3d & input) const
{
	Vector3d resultVector;

	switch (m_axis)
	{
	default:
	case 'x':
		return Vector3d(input.x, -input.y, input.z);
		break;
	case 'y':
		return Vector3d(-input.x, input.y, input.z);
		break;
	}
}
