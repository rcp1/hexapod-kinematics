#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "kinematicControl.h"
#include "defines.h"
#include "interrupt.h"
#include "gaitHandler.h"
#include "bodyHandler.h"
#include "task.h"
#include "interpolationHandler.h"
#include "movingAverage.h"

enum commands
{
    xPlus = 'q',
    xMinus = 'e',
    yPlus = 'a',
    yMinus = 'd',
    zPlus = 'w',
    zMinus = 's',
    psiPlus = 'u',
    psiMinus = 'o',
    thetaPlus = 'i',
    thetaMinus = 'k',
    phiPlus = 'j',
    phiMinus = 'l',
    speedPlus = 't',
    speedMinus = 'g',
    swingLengthPlus = 'y',
    swingLengthMinus = 'x',
    turnDistancePlus = 'f',
    turnDistanceMinus = 'h',
    tripod = 'c',
    wave = 'v',
    start = 'b',
    lay = 'n',
    standUp = 'm',
    complex = 'r',
    complexSwingLength = 'z'
};

enum taskPriority
{
    taskServo,
    taskInput,
    taskOutput,
    freeRunning
};

class Scheduler
{
public:
    Scheduler();

    ~Scheduler();

    void run();

protected:
    void transition();

    uint8_t taskDecision();

    void getSerialInput();

    void parseComplexCmd();

    float getFloatFromPercentCmd(const uint16_t& i, const float& limit);

    void readCommand();

    static const uint8_t m_bluetoothDelimiter;
    static const float m_increaseBodyTranslationFactor; ///> m
    static const float m_increaseBodyRotationFactor; ///> rad
    static const float m_increaseSpeedFactor; ///> m/s
    static const float m_increaseSwingLength; ///> percent
    static const float m_increaseTurnAngle; ///> m

    KinematicControl m_kinControl;
    GaitHandler m_gaitHandler;
    BodyHandler m_bodyHandler;
    InterpolationHandler m_interpolationHandler;

    STRING m_readData;
    uint8_t m_crc;
    float m_turnAngle; // %

    Task m_taskServo;
    Task m_taskInput;
    Task m_TaskOutput;

    MovingAverage<10> m_turnAv;
};

#endif // SCHEDULER_H
