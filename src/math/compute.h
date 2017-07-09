#ifndef COMPUTE_H
#define COMPUTE_H

#include <stdint.h>

namespace math
{

inline
float power(float value, uint8_t power)
{
    float result = value;

    for (uint8_t i = 0; i <= (power - (uint8_t)(2)); ++i)
    {
        result *= value;
    }

    return result;
}

}

#endif // COMPUTE_H
