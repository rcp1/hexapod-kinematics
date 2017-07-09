#include "orientation3d.h"
#include "mathConstants.h"
#include <math.h>
#include <string.h>

Orientation3d::Orientation3d(float psi, float theta, float phi) :
	psi(psi),
	theta(theta),
	phi(phi)
{
}

Orientation3d::Orientation3d(const float* val, uint8_t dimension)
{
	memset(&m_data, 0, s_dimension * sizeof(float));

	for (uint8_t i = 0; i < dimension; ++i)
	{
		this->m_data[i] = val[i];
	}
}

Orientation3d::Orientation3d(const Orientation3d& other)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] = other.m_data[i];
	}
}

uint8_t Orientation3d::isBigger(const Orientation3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] > source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Orientation3d::isSmaller(const Orientation3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if (this->m_data[i] < source.m_data[i])
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

uint8_t Orientation3d::isEqual(const Orientation3d& source) const
{
	uint8_t resultBitmap = 0;

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		if ((float)fabs(this->m_data[i] - source.m_data[i]) < math::epsilonFloat)
			resultBitmap |= (0x01 << i);
	}

	return resultBitmap;
}

Orientation3d& Orientation3d::operator+=(const Orientation3d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] += source.m_data[i];
	}

	return *this;
}

Orientation3d& Orientation3d::operator-=(const Orientation3d& source)
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		this->m_data[i] -= source.m_data[i];
	}

	return *this;
}

Orientation3d Orientation3d::operator/(const float scalar) const
{
	if (scalar < math::epsilonFloat)
	{
		// Division through zero
		return *this;
	}

	Orientation3d temp(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		temp.m_data[i] /= scalar;
	}

	return temp;
}

Orientation3d& Orientation3d::operator/=(const float scalar)
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

Orientation3d Orientation3d::operator*(const float scalar) const
{
	Orientation3d temp(*this);

	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		temp.m_data[i] *= scalar;
	}

	return temp;
}

Orientation3d& Orientation3d::operator*=(const float scalar)
{
	for (uint8_t i = 0; i < this->s_dimension; ++i)
	{
		this->m_data[i] *= scalar;
	}

	return *this;
}

void Orientation3d::clear()
{
	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		m_data[i] = 0.f;
	}
}
