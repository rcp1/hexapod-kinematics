#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "helper.h"
#include <stdint.h>

class Vector3d
{
public:
	explicit Vector3d(float x = 0.0f, float y = 0.0f, float z = 0.0f);

	Vector3d(const float* val, uint8_t dimension);

	Vector3d(const Vector3d& other);

	Vector3d& operator=(Vector3d other);

	uint8_t isBigger(const Vector3d& source) const;

	uint8_t isSmaller(const Vector3d& source) const;

	uint8_t isEqual(const Vector3d& source) const;

	Vector3d operator+(const Vector3d& source) const;

	Vector3d& operator+=(const Vector3d& source);

	Vector3d operator-(const Vector3d& source) const;

	Vector3d& operator-=(const Vector3d& source);

	Vector3d operator/(const float scalar) const;

	Vector3d& operator/=(const float scalar);

	Vector3d operator*(const float scalar) const;

	Vector3d& operator*=(const float scalar);

	float operator*(const Vector3d& source) const;

	float dot(const Vector3d& source) const;

	Vector3d cross(const Vector3d& source) const;

	float& operator[](uint8_t index);

	float operator[](uint8_t index) const;

	void norm();

	float length();

	void clear();

	union
	{
		struct
		{
			float x;
			float y;
			float z;
		};
		float m_data[3];
	};

	static const uint8_t s_dimension = 3;
};

inline
Vector3d& Vector3d::operator=(Vector3d other)
{
    swap<float>(this->m_data, other.m_data, s_dimension);

    return *this;
}

inline
Vector3d Vector3d::operator+(const Vector3d& source) const
{
	Vector3d tempVec(*this);

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		tempVec.m_data[i] += source.m_data[i];
	}

	return tempVec;
}

inline
Vector3d Vector3d::operator-(const Vector3d& source) const
{
	Vector3d tempVec(*this);

	for (uint8_t i = 0; i < s_dimension; ++i)
	{
		tempVec.m_data[i] -= source.m_data[i];
	}

	return tempVec;
}

inline
float& Vector3d::operator[](uint8_t index)
{
	return m_data[index];
}

inline
float Vector3d::operator[](uint8_t index) const
{
	return m_data[index];
}

#endif // VECTOR3D_H
