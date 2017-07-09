#ifndef POSE3D_H
#define POSE3D_H

#include "vector3d.h"
#include "orientation3d.h"

class Pose3d
{
public:
    Pose3d();

    virtual ~Pose3d();

    explicit Pose3d(Vector3d position, Orientation3d orientation);

    Pose3d(const float* val, uint8_t dimension);

    Pose3d(const Pose3d& other);

    Pose3d& operator=(Pose3d other);

    Pose3d operator+(Pose3d source) const;

    Pose3d& operator+=(const Pose3d& source);

    Pose3d operator+(const Vector3d& source) const;

    Pose3d& operator+=(const Vector3d& source);

    Pose3d operator+(const Orientation3d& source) const;

    Pose3d& operator+=(const Orientation3d& source);

    Pose3d operator-(const Pose3d& source) const;

    Pose3d& operator-=(const Pose3d& source);

    Pose3d operator-(const Vector3d& source) const;

    Pose3d& operator-=(const Vector3d& source);

    Pose3d operator-(const Orientation3d& source) const;

    Pose3d& operator-=(const Orientation3d& source);

    float& operator[](uint8_t index);

    float operator[](uint8_t index) const;

    Vector3d m_position;
    Orientation3d m_orientation;
};


inline
Pose3d& Pose3d::operator=(Pose3d other)
{
    this->m_position = other.m_position;
    this->m_orientation = other.m_orientation;

    return *this;
}

inline
Pose3d Pose3d::operator+(Pose3d source) const
{
    source.m_position += this->m_position;
    source.m_orientation += this->m_orientation;

    return source;
}


inline
Pose3d Pose3d::operator+(const Vector3d& source) const
{
    Pose3d temp(*this);

    temp.m_position += source;

    return temp;
}

inline
Pose3d Pose3d::operator+(const Orientation3d& source) const
{
    Pose3d temp(*this);

    temp.m_orientation += source;

    return temp;
}

inline
Pose3d Pose3d::operator-(const Pose3d& source) const
{
    Pose3d temp(*this);

    temp.m_position -= source.m_position;
    temp.m_orientation -= source.m_orientation;

    return temp;
}

inline
Pose3d Pose3d::operator-(const Vector3d& source) const
{
    Pose3d temp(*this);

    temp.m_position -= source;

    return temp;
}


inline
Pose3d Pose3d::operator-(const Orientation3d& source) const
{
    Pose3d temp(*this);

    temp.m_orientation -= source;

    return temp;
}

inline
float& Pose3d::operator[](uint8_t index)
{
    if(index < Vector3d::s_dimension)
    {
        return m_position[index];
    }
    else
    {
        return m_orientation[index - Vector3d::s_dimension];
    }
}

inline
float Pose3d::operator[](uint8_t index) const
{
    if(index < Vector3d::s_dimension)
    {
        return m_position[index];
    }
    else
    {
        return m_orientation[index - Vector3d::s_dimension];
    }
}

#endif // POSE3D_H
