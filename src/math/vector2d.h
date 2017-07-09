#ifndef VECTOR2D_H
#define VECTOR2D_H

#include "helper.h"
#include <stdint.h>

class Vector2d
{
public:
    explicit Vector2d(float x = 0.0f, float y = 0.0f);

    Vector2d(const float* val, uint8_t dimension);

    Vector2d(const Vector2d& other);

    Vector2d& operator=(Vector2d other);

    uint8_t isBigger(const Vector2d& source) const;

    uint8_t isSmaller(const Vector2d& source) const;

    uint8_t isEqual(const Vector2d& source) const;

    Vector2d operator+(const Vector2d& source) const;

    Vector2d& operator+=(const Vector2d& source);

    Vector2d operator-(const Vector2d& source) const;

    Vector2d& operator-=(const Vector2d& source);

    Vector2d operator/(const float scalar) const;

    Vector2d& operator/=(const float scalar);

    Vector2d operator*(const float scalar) const;

    Vector2d& operator*=(const float scalar);

    float operator*(const Vector2d& source) const;

    float dot(const Vector2d& source) const;

    float cross(const Vector2d& source) const;

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
        };
        float m_data[2];
    };

    static const uint8_t s_dimension = 2;
};

inline
Vector2d& Vector2d::operator=(Vector2d other)
{
    swap<float>(this->m_data, other.m_data, s_dimension);

    return *this;
}

inline
Vector2d Vector2d::operator+(const Vector2d& source) const
{
    Vector2d tempVec(*this);

    for (uint8_t i = 0; i < s_dimension; ++i)
    {
        tempVec.m_data[i] += source.m_data[i];
    }

    return tempVec;
}

inline
Vector2d Vector2d::operator-(const Vector2d& source) const
{
    Vector2d tempVec(*this);

    for (uint8_t i = 0; i < s_dimension; ++i)
    {
        tempVec.m_data[i] -= source.m_data[i];
    }

    return tempVec;
}

inline
float& Vector2d::operator[](uint8_t index)
{
    return m_data[index];
}

inline
float Vector2d::operator[](uint8_t index) const
{
    return m_data[index];
}

#endif // VECTOR2D_H
