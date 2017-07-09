#include "pose3d.h"
#include <string.h>

Pose3d::Pose3d()
{
}

Pose3d::~Pose3d()
{
}

Pose3d::Pose3d(Vector3d position, Orientation3d orientation) :
	m_position(position),
	m_orientation(orientation)
{
}

Pose3d::Pose3d(const float* val, uint8_t dimension)
{
	memset(&m_position, 0, Vector3d::s_dimension * sizeof(float));
	memset(&m_orientation, 0, Orientation3d::s_dimension * sizeof(float));

	for (uint8_t i = 0; i < dimension; ++i)
	{
		if(i < Vector3d::s_dimension)
		{
			this->m_position[i] = val[i];
		}
		else
		{
			this->m_orientation[i - Vector3d::s_dimension] = val[i - Vector3d::s_dimension];
		}
	}
}

Pose3d::Pose3d(const Pose3d& other)
{
	this->m_position = other.m_position;
	this->m_orientation = other.m_orientation;
}

Pose3d& Pose3d::operator+=(const Pose3d& source)
{
	this->m_position += source.m_position;
	this->m_orientation += source.m_orientation;

	return *this;
}

Pose3d& Pose3d::operator+=(const Vector3d& source)
{
	this->m_position += source;

	return *this;
}

Pose3d& Pose3d::operator+=(const Orientation3d& source)
{
	this->m_orientation += source;

	return *this;
}

Pose3d& Pose3d::operator-=(const Pose3d& source)
{
	this->m_position -= source.m_position;
	this->m_orientation -= source.m_orientation;

	return *this;
}

Pose3d& Pose3d::operator-=(const Vector3d& source)
{
	this->m_position -= source;

	return *this;
}

Pose3d& Pose3d::operator-=(const Orientation3d& source)
{
	this->m_orientation -= source;

	return *this;
}
