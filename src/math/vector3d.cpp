#include "vector3d.h"
#include "mathConstants.h"
#include <math.h>
#include <string.h>

Vector3d::Vector3d(float x, float y, float z) :
	x(x),
	y(y),
	z(z)
{
}

Vector3d::Vector3d(const float* val, uint8_t dimension)
{
	memset(&m_data, 0, s_dimension * sizeof(float));

	for (uint8_t i = 0; i < dimension; ++i)
	{
		this->m_data[i] = val[i];
	}
}

Vector3d::Vector3d(const Vector3d& other)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] = other.m_data[i];
	}
}

uint8_t Vector3d::isBigger(const Vector3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] > source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Vector3d::isSmaller(const Vector3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] < source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Vector3d::isEqual(const Vector3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if ((float)fabs(this->m_data[i] - source.m_data[i]) < math::epsilonFloat)
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

Vector3d& Vector3d::operator+=(const Vector3d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] += source.m_data[i];
	}

	return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] -= source.m_data[i];
	}

	return *this;
}

Vector3d Vector3d::operator/(const float scalar) const
{
	if (scalar < math::epsilonFloat)
	{
		// Division through zero
		return *this;
	}

	Vector3d tempVec(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		tempVec.m_data[i] /= scalar;
	}

	return tempVec;
}

Vector3d& Vector3d::operator/=(const float scalar)
{
	if (scalar < math::epsilonFloat)
	{
		// Division through zero
		return *this;
	}

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		this->m_data[i] /= scalar;
	}

	return *this;
}

Vector3d Vector3d::operator*(const float scalar) const
{
	Vector3d tempVec(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		tempVec.m_data[i] *= scalar;
	}

	return tempVec;
}

Vector3d& Vector3d::operator*=(const float scalar)
{
	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		this->m_data[i] *= scalar;
	}

	return *this;
}

float Vector3d::operator*(const Vector3d& source) const
{
	// scalar product
	return dot(source);
}

float Vector3d::dot(const Vector3d& source) const
{
	float dotResult;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		dotResult += this->m_data[i] * source.m_data[i];
	}

	return dotResult;
}

Vector3d Vector3d::cross(const Vector3d& source) const
{
	Vector3d crossedVec;

	crossedVec.x = this->y * source.z - this->z * source.y;
	crossedVec.y = this->z * source.x - this->x * source.z;
	crossedVec.z = this->x * source.y - this->y * source.x;

	return crossedVec;
}

void Vector3d::norm()
{
	float length = this->length();

	if (length < math::epsilonFloat)
	{
		return;
	}

	*this /= length;
}

float Vector3d::length()
{
	float squareRoot = 0.f;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		squareRoot += (m_data[i] * m_data[i]);
	}
	squareRoot = sqrtf(squareRoot);

	return squareRoot;
}

void Vector3d::clear()
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		m_data[i] = 0.f;
	}
}
