#include "trafoCoordRotation.h"
#include <math.h>

TrafoRotation::TrafoRotation(uint8_t axis, float angleForward) : m_axis(axis), m_angleForward(angleForward)
{
}

TrafoRotation::~TrafoRotation()
{
}

trafoStatus TrafoRotation::forward(const Vector3d& input, Vector3d& output) const
{
    trafoStatus result = trafoOk;

    result = rotate(input, output, m_angleForward);
    return result;
}

trafoStatus TrafoRotation::backward(const Vector3d& input, Vector3d& output) const
{
    trafoStatus result = trafoOk;

    result = rotate(input, output, -m_angleForward);
    return result;
}

void TrafoRotation::setAxisAndAngle(const uint8_t& axis, const float& angleForward)
{
    this->m_axis = axis;
    this->m_angleForward = angleForward;
}

trafoStatus TrafoRotation::rotate(const Vector3d& input, Vector3d& output, const float& angle) const
{
    trafoStatus result = trafoOk;
    Vector3d resultVector;

    switch (m_axis)
    {
    case 'x':
        resultVector.x =                      input.x/*+ 0                * input.y    +        0            * input.z*/;
        resultVector.y =  /*0                * input.x +*/cosf(angle)    * input.y - sinf(angle)    * input.z;
        resultVector.z =  /*0                * input.x +*/sinf(angle)    * input.y + cosf(angle)    * input.z;
        break;
    case 'y':
        resultVector.x = cosf(angle)        * input.x +     /*0            * input.y*/        sinf(angle)    * input.z;
        resultVector.y =  /*0                * input.x +                   1*/input.y /* +        0            * input.z */;
        resultVector.z = -sinf(angle)    * input.x +     /*0            * input.y*/        cosf(angle)    * input.z;
        break;
    case 'z':
        resultVector.x = cosf(angle)        * input.x - sinf(angle)    * input.y /* +        0            * input.z */;
        resultVector.y = sinf(angle)        * input.x + cosf(angle)    * input.y /* +        0            * input.z */;
        resultVector.z =  /*0                * input.x +    0                * input.y    +        1            */input.z;
        break;
    }
    output = resultVector;
    return result;
}
