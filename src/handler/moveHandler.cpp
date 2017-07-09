#include "moveHandler.h"

MoveHandler::MoveHandler(KinematicControl& kinematicControl, uint32_t interval) :
    m_kinematicControl(kinematicControl),
    m_interval(interval),
    m_countMoves(0),
    m_actualMove(0),
    m_speed(0.f),
    m_isBlocked(false),
    m_isFinished(false)
{
}

MoveHandler::~MoveHandler()
{
}

void MoveHandler::setSpeed(const float& speed)
{
    if (!m_isBlocked)
    {
        m_speed = speed;
        float swingTime = m_kinematicControl.calcSwingTime(m_speed);

        calcCountOfMoves(swingTime);
    }
}

void MoveHandler::calcCountOfMoves(const float& swingTime)
{
    float actualTime = getActualTime();
    m_countMoves = (uint8_t)roundf((float)((swingTime / (float)m_interval)));
    setActualMove((uint8_t)roundf(actualTime * (float)m_countMoves));
}
