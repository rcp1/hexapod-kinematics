#include "vector2d.h"
#include "mathConstants.h"
#include <math.h>
#include <string.h>

void swap(Vector2d& first, const Vector2d& second)
{
	for (uint8_t i = 0; i < first.s_dimension; ++i)
	{
		first.m_data[i] = second.m_data[i];
	}
}

Vector2d::Vector2d(float x, float y) :
	x(x),
	y(y)
{
}

Vector2d::Vector2d(const float* val, uint8_t dimension)
{
	memset(&m_data, 0, s_dimension * sizeof(float));

	for (uint8_t i = 0; i < dimension; ++i)
	{
		this->m_data[i] = val[i];
	}
}

uint8_t Vector2d::isBigger(const Vector2d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] > source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Vector2d::isSmaller(const Vector2d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] < source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Vector2d::isEqual(const Vector2d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if ((float)fabs(this->m_data[i] - source.m_data[i]) < math::epsilonFloat)
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

Vector2d& Vector2d::operator+=(const Vector2d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] += source.m_data[i];
	}

	return *this;
}

Vector2d& Vector2d::operator-=(const Vector2d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] -= source.m_data[i];
	}

	return *this;
}

Vector2d Vector2d::operator/(const float scalar) const
{
	if (scalar < math::epsilonFloat)
	{
		// Division through zero
		return *this;
	}

	Vector2d tempVec(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		tempVec.m_data[i] /= scalar;
	}

	return tempVec;
}

Vector2d& Vector2d::operator/=(const float scalar)
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

Vector2d Vector2d::operator*(const float scalar) const
{
	Vector2d tempVec(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		tempVec.m_data[i] *= scalar;
	}

	return tempVec;
}

Vector2d& Vector2d::operator*=(const float scalar)
{
	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		this->m_data[i] *= scalar;
	}

	return *this;
}

float Vector2d::operator*(const Vector2d& source) const
{
	// scalar product
	return dot(source);
}

float Vector2d::dot(const Vector2d& source) const
{
	float dotResult;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		dotResult += this->m_data[i] * source.m_data[i];
	}

	return dotResult;
}

float Vector2d::cross(const Vector2d& source) const
{
	return (this->x * source.y - this->y * source.x);
}

void Vector2d::norm()
{
	float length = this->length();

	if (length < math::epsilonFloat)
	{
		return;
	}

	*this /= length;
}

float Vector2d::length()
{
	float squareRoot = 0.f;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		squareRoot += (m_data[i] * m_data[i]);
	}
	squareRoot = sqrtf(squareRoot);

	return squareRoot;
}

void Vector2d::clear()
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		m_data[i] = 0.f;
	}
}
