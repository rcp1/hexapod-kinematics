#include "scheduler.h"
#include "helper.h"
#include "angles.h"

const uint8_t Scheduler::m_bluetoothDelimiter = (uint8_t)'\n';
const float Scheduler::m_increaseBodyTranslationFactor = (float)0.005f;
const float Scheduler::m_increaseBodyRotationFactor = (float)math::deg2rad(3);
const float Scheduler::m_increaseSpeedFactor = (float)0.005;
const float Scheduler::m_increaseSwingLength = (float)0.05f;
const float Scheduler::m_increaseTurnAngle = (float)0.05f;

Scheduler::Scheduler() :
    m_kinControl(m_interpolationHandler, m_gaitHandler, m_bodyHandler),
    m_gaitHandler(m_kinControl, tasks::servoInterval, gaits::type::tripod),
    m_bodyHandler(m_kinControl, tasks::servoInterval),
    m_crc(0),
    m_turnAngle(0.f),
    m_taskServo(tasks::servoInterval),
    m_taskInput(tasks::inputInterval),
    m_TaskOutput(tasks::outputInterval)
{
    m_bodyHandler.setSpeed(speeds::body::normal);
    m_kinControl.setInitialBodyPose(Pose3d(bodyVector::lay, Vector3d::s_dimension));
    m_kinControl.calcInterpolationToStart(gaits::type::tripod);
    m_bodyHandler.setBlocking(true);
    m_gaitHandler.setBlocking(true);
}

Scheduler::~Scheduler()
{
}

void Scheduler::run()
{
    // calculate first position
    m_kinControl.calcKin();
    this->transition();
}

void Scheduler::transition()
{
    while (1)
    {
        switch (taskDecision())
        {
        case taskServo:
            // set old position
            m_kinControl.setServos();
            // transit to next step
            m_gaitHandler.transition();
            m_bodyHandler.transition();
            // calculate next position
            m_kinControl.calcKin();

            m_taskServo.finished();
            break;
        case taskInput:
            getSerialInput();
            m_taskInput.finished();
            break;
        case taskOutput:
            Serial.print("Body: ");
            Serial.print(m_kinControl.getBodyPose().m_position.x, 4);
            Serial.print(" | ");
            Serial.print(m_kinControl.getBodyPose().m_position.y, 4);
            Serial.print(" | ");
            Serial.print(m_kinControl.getBodyPose().m_position.z, 4);
            Serial.print(" | ");
            Serial.print(math::rad2deg(m_kinControl.getBodyPose().m_orientation.psi), 4);
            Serial.print(" | ");
            Serial.print(math::rad2deg(m_kinControl.getBodyPose().m_orientation.phi), 4);
            Serial.print(" | ");
            Serial.println(math::rad2deg(m_kinControl.getBodyPose().m_orientation.theta), 4);
            m_TaskOutput.finished();
            break;
        case freeRunning:
            break;
        }
    }
}

uint8_t Scheduler::taskDecision()
{
    if (m_taskServo.isActive())
    {
        return taskServo;
    }

    if (m_taskInput.isActive() && !m_taskServo.isActive())
    {
        return taskInput;
    }

    if (m_TaskOutput.isActive() && !m_taskServo.isActive() && !m_taskInput.isActive())
    {
        return taskOutput;
    }

    return taskInput;
}

void Scheduler::getSerialInput()
{
    readCommand();

    char actState_uc = m_readData[0];
    Pose3d bodyVector = m_kinControl.getBodyPose();
    switch ((enum commands)actState_uc)
    {
    case xPlus:
        bodyVector.m_position.x += m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case xMinus:
        bodyVector.m_position.x -= m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case yPlus:
        bodyVector.m_position.y += m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case yMinus:
        bodyVector.m_position.y -= m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case zPlus:
        bodyVector.m_position.z += m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case zMinus:
        bodyVector.m_position.z -= m_increaseBodyTranslationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case psiPlus:
        bodyVector.m_orientation.psi += m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case psiMinus:
        bodyVector.m_orientation.psi -= m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case thetaPlus:
        bodyVector.m_orientation.theta += m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case thetaMinus:
        bodyVector.m_orientation.theta -= m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case phiPlus:
        bodyVector.m_orientation.phi += m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case phiMinus:
        bodyVector.m_orientation.phi -= m_increaseBodyRotationFactor;
        m_kinControl.setNewBodyPose(bodyVector);
        break;
    case speedPlus:
        m_gaitHandler.setSpeed(m_gaitHandler.getSpeed() + m_increaseSpeedFactor);
        break;
    case speedMinus:
        m_gaitHandler.setSpeed(m_gaitHandler.getSpeed() - m_increaseSpeedFactor);
        break;
    case swingLengthPlus:
        m_kinControl.setSwingLength(minimum(m_kinControl.getSwingLength() + m_increaseSwingLength, 1.f));
        break;
    case swingLengthMinus:
        m_kinControl.setSwingLength(maximum(m_kinControl.getSwingLength() - m_increaseSwingLength, 0.f));
        break;
    case turnDistancePlus:
        m_turnAngle = minimum(m_turnAngle + m_increaseTurnAngle, 1.f);
        m_kinControl.setTurnAngle(m_turnAngle);
        break;
    case turnDistanceMinus:
        m_turnAngle = maximum(m_turnAngle - m_increaseTurnAngle, -1.f);
        m_kinControl.setTurnAngle(m_turnAngle);
        break;
    case start:
        m_gaitHandler.setGait(gaits::type::target);
        m_gaitHandler.setSpeed(speeds::gaits::target);
        m_kinControl.calcInterpolationToInit();
        break;
    case tripod:
        m_gaitHandler.setGait(gaits::type::tripod);
        break;
    case wave:
        m_gaitHandler.setGait(gaits::type::wave);
        break;
    case lay:
        m_kinControl.setNewBodyPose(Pose3d(bodyVector::lay, Vector3d::s_dimension));
        m_bodyHandler.setSpeed(speeds::body::layStand);
        m_bodyHandler.setBlocking(true);
        m_gaitHandler.setBlocking(true);
        break;
    case standUp:
        m_bodyHandler.setBlocking(false);
        m_kinControl.setNewBodyPose(Pose3d(bodyVector::stand, Vector3d::s_dimension));
        m_bodyHandler.setSpeed(speeds::body::layStand);
        m_gaitHandler.setBlocking(false);
        break;
    case complex:
        parseComplexCmd();
        break;
    case complexSwingLength:
        m_kinControl.setSwingLength(limit(0.f, (float)m_readData.c_str()[1] / 100.f, 1.f));
        break;
    default:
        break;
    }
}

void Scheduler::parseComplexCmd()
{
    if (m_gaitHandler.getGait() != gaits::target)
    {
        m_gaitHandler.setSpeed(getFloatFromPercentCmd(1, speeds::gaits::limit));
    }
    else
    {
        m_gaitHandler.setSpeed(getFloatFromPercentCmd(1, speeds::gaits::target));
    }

    m_turnAv.addValue(getFloatFromPercentCmd(2, 1.f));
    m_turnAngle = limit(-1.f, m_turnAv.getAverage(), 1.f);
    m_kinControl.setTurnAngle(m_turnAngle);

    Pose3d bodyVector = m_kinControl.getBodyPose();
    bodyVector.m_orientation.phi = getFloatFromPercentCmd(3, bodyVector::limits::rotation);
    bodyVector.m_orientation.theta = getFloatFromPercentCmd(4, bodyVector::limits::rotation);
    bodyVector.m_orientation.psi = getFloatFromPercentCmd(5, bodyVector::limits::rotation);
    bodyVector.m_position.x = getFloatFromPercentCmd(6, bodyVector::limits::translation);
    bodyVector.m_position.y = getFloatFromPercentCmd(7, bodyVector::limits::translation);
    bodyVector.m_position.z = getFloatFromPercentCmd(8, bodyVector::limits::translation);
    m_bodyHandler.setSpeed(speeds::body::normal);
    m_kinControl.setNewBodyPose(bodyVector);

}

float Scheduler::getFloatFromPercentCmd(const uint16_t& i, const float& limit)
{
    return ((float)(m_readData[i] - 50) / 50.f) * limit;
}

#ifdef ARDUINO_PLATFORM
void Scheduler::readCommand()
{
    m_readData = "";

    if (Serial1.available() >= 11)
    {
        while (Serial1.available())
        {
            char ch = Serial1.read();
            if (ch == -1)
            {
                // Handle error
            }
            else if ((uint8_t)m_readData.charAt(m_readData.length() - 1) == m_crc && ch == '\n')
            {
                m_crc++;
                break;
            }
            else
            {
                m_readData += ch;
            }
        }
        Serial1.flush();
        Serial1.write((uint8_t)m_crc);
    }
}
#endif // ARDUINO_PLATFORM
