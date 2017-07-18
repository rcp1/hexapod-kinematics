#ifndef DEFINES_H
#define DEFINES_H

#ifndef ARDUINO_PLATFORM
#define ARDUINO_PLATFORM
#endif

#include "vector3d.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

typedef String STRING;

const float degToRadFactor = 0.01745329251994329f; ///> pi / 180
const float radToDegFactor = 1.f / degToRadFactor; ///> 180 / pi

const uint16_t servoNullOffset = static_cast<uint16_t>(1500);
const uint16_t servoMinimum = static_cast<uint16_t>(500);
const uint16_t servoMaximum = static_cast<uint16_t>(2500);

const float radToServoFactor = 1000.f / HALF_PI;
const float servoToRadFactor = 1.f / radToServoFactor;

const uint32_t bluetoothBaudrate = 115200UL; ///> PIN/PW: 1100
const uint32_t serialBaudrate = 115200UL;

// Define the used servo controller
#define SSC32
// #define SD21

#ifdef SD21
const uint8_t sd21AddressI2C = static_cast<uint8_t>0x61; //> shifted by one bit from 0xC2
const uint8_t sd21RefreshRate = static_cast<uint8_t>20; ///> [ms]
const uint8_t sd21CountRegistersPerLeg = static_cast<uint8_t>9;
#endif

#ifdef SSC32
const uint32_t ssc32Baudrate = 115200UL;
#endif

const float toleranceTurn = 0.005f;

namespace tV
{
const uint8_t maxDimensionOrientation = static_cast<uint8_t>(3);
const uint8_t maxDimensionPose = Vector3d::s_dimension + maxDimensionOrientation;
} // namespace tV

namespace msrh01
{
const uint8_t legs = static_cast<uint8_t>(6);
const uint8_t servosPerLeg = static_cast<uint8_t>(3);

const uint8_t tibiaIndex = static_cast<uint8_t>(0);
const uint8_t femurIndex = static_cast<uint8_t>(1);
const uint8_t coxaIndex = static_cast<uint8_t>(2);

const float anglesInit[servosPerLeg] = {0.0f, 0.0f, 0.0f};

const int16_t servoOffsets[legs * servosPerLeg] =
{
// Tibia, Femur, Coxa
    -50, 50, 0, // fr
    -20, -100, -70, // fl
    -90, -40, 30, // mr
    -40, 20, 90, // ml
    -70, -40, 20, // br
     20, -80, -60 // bl
};

enum legIndex
{
  fr = 0,
  fl,
  mr,
  ml,
  br,
  bl,
};

namespace dimensions
{
const float femur = 0.08f; ///> [m]
const float tibia = 0.13f; ///> [m]
const float coxa = 0.0266f; ///> [m]
const float bodyToCornerLegsX = 0.08f; ///> [m]
const float bodyToCornerLegsY = 0.04f; ///> [m]
const float bodyToMidLegsX = 0.f; ///> [m]
const float bodyToMidLegsY = 0.06f; ///> [m]
} // namespace dimensions

namespace ANGLES
{
const float fr = (30.f * degToRadFactor);  ///> [rad]
const float fl = (150.f * degToRadFactor); ///> [rad]
const float mr = (0.f * degToRadFactor); ///> [rad]
const float ml = (180.f * degToRadFactor); ///> [rad]
const float br = (330.f * degToRadFactor); ///> [rad]
const float bl = (210.f * degToRadFactor); ///> [rad]
} // namespace ANGLES

const float stepLengthMaximum = sinf(25.f * degToRadFactor) *
                                (dimensions::coxa + dimensions::femur); ///> [m]
} // namespace msrh01

namespace gaits
{
enum type { target, tripod, wave, ripple, gaitCount };
enum state { stop, targetSwing, stance, swing, stateCount };

const uint8_t START_STATES[gaitCount][msrh01::legs] =
{
    targetSwing, stop, stop, stop, stop, stop, // target
    swing, stance, stance, swing,  swing,  stance, // tripod
    swing, stance, stance, stance, stance, stance, // wave
    swing, stance, stance, stance, stance, stance // ripple
};

} // namespace gaits

namespace tasks
{
const uint16_t servoInterval = static_cast<uint16_t>(100); ///> [ms]
const uint16_t inputInterval = static_cast<uint16_t>(50);  ///> [ms]
const uint16_t outputInterval = static_cast<uint16_t>(500); ///> [ms]
} // namespace tasks

namespace bodyVector
{
const float lay[Vector3d::s_dimension] = {0.0f, 0.0f, -0.105f}; ///> [m]
const float stand[Vector3d::s_dimension] = {0.0f, 0.0f, 0.0f};  ///> [m]

namespace limits
{
const float translation = 0.05f; ///> [m]
const float rotation = degToRadFactor * 15.0f; ///> [rad]
} // namespace limits

} // namespace bodyVector

namespace speeds
{

namespace body
{
const float normal = 0.05f; ///> [m/s]
const float layStand = 0.01f; ///> [m/s]
} // namespace body

namespace gaits
{
const float limit = 0.035f; ///> [m/s]
const float target = 0.3f; ///> [m/s]
} // namespace gaits

} // namespace speeds

#endif // DEFINES_H
