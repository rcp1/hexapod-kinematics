#ifndef ORIENTATION3D_H
#define ORIENTATION3D_H

#include "helper.h"
#include <stdint.h>

class Orientation3d
{
public:
	explicit Orientation3d(float psi = 0.0f, float theta = 0.0f, float phi = 0.0f);

	Orientation3d(const float* val, uint8_t dimension);

	Orientation3d(const Orientation3d& other);

	Orientation3d& operator=(Orientation3d other);

	uint8_t isBigger(const Orientation3d& source) const;

	uint8_t isSmaller(const Orientation3d& source) const;

	uint8_t isEqual(const Orientation3d& source) const;

	Orientation3d operator+(const Orientation3d& source) const;

	Orientation3d& operator+=(const Orientation3d& source);

	Orientation3d operator-(const Orientation3d& source) const;

	Orientation3d& operator-=(const Orientation3d& source);

	Orientation3d operator/(const float scalar) const;

	Orientation3d& operator/=(const float scalar);

	Orientation3d operator*(const float scalar) const;

	Orientation3d& operator*=(const float scalar);

	float& operator[](uint8_t index);

	float operator[](uint8_t index) const;

	void clear();

	union
	{
		struct
		{
			float psi;
			float theta;
			float phi;
		};
		float m_data[3];
	};

	static const uint8_t s_dimension = 3;
};

inline
Orientation3d& Orientation3d::operator=(Orientation3d other)
{
	swap<float>(this->m_data, other.m_data, Orientation3d::s_dimension);

    return *this;
}

inline
Orientation3d Orientation3d::operator+(const Orientation3d& source) const
{
	Orientation3d temp(*this);

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		temp.m_data[i] += source.m_data[i];
	}

	return temp;
}

inline
Orientation3d Orientation3d::operator-(const Orientation3d& source) const
{
	Orientation3d temp(*this);

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		temp.m_data[i] -= source.m_data[i];
	}

	return temp;
}

inline
float& Orientation3d::operator[](uint8_t index)
{
	return m_data[index];
}

inline
float Orientation3d::operator[](uint8_t index) const
{
	return m_data[index];
}

#endif // ORIENTATION3D_H
