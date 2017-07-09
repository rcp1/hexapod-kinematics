#ifndef SERVOTYPES_H
#define SERVOTYPES_H

#include "defines.h"
#include <stdint.h>

struct ServoPosition
{
    union
    {
        struct
        {
            uint8_t lowerPos;
            uint8_t upperPos;
        };
        uint16_t pos;
    };
    union
    {
        struct
        {
            uint8_t latchedLowerPos;
            uint8_t latchedUpperPos;
        };
        uint16_t latchedPos;
    };
};

class LegServoVector
{
public:
    LegServoVector(const uint8_t legIndex = 1, const uint16_t moveTime = 0);

    LegServoVector(const Vector3d& input, const uint16_t moveTime = 0);

    void init();

    void setTrafoVector(const Vector3d& input);

    Vector3d getTrafoVector();

    void setLegIndex(const uint8_t legIndex);

    void setMoveTime(const uint16_t moveTime);

    void setServos();

protected:
// Servo controller handle methods
#ifdef SSC32
    void setServosSSC32();
#endif
#ifdef SD21
    void setServosSD21();
#endif

    uint8_t m_legIndex;
    uint16_t m_moveTime;
    ServoPosition m_servoVectors[msrh01::servosPerLeg];
    Vector3d m_ACSTCP;
};

#endif // SERVOTYPES_H
