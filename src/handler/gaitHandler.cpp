#include "gaitHandler.h"
#include "mathConstants.h"

GaitHandler::GaitHandler(KinematicControl& kinematicControl, uint32_t interval, uint8_t gaitNr) :
    MoveHandler(kinematicControl, interval),
    m_gaitNrAfterTarget(0)
{
    setGait(gaitNr);
}

GaitHandler::~GaitHandler()
{
}

void GaitHandler::setStartStates()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_actualLegStates[i] = gaits::START_STATES[m_gaitNr][i];
    }
}

uint8_t GaitHandler::getStartStates(uint8_t gaitNr, uint8_t legIndex)
{
    return gaits::START_STATES[gaitNr][legIndex];
}

void GaitHandler::transition()
{
    if (m_speed > math::epsilonFloat && !m_isBlocked)
    {
        m_actualMove++;

        if (m_actualMove > m_countMoves)
        {
            switchPhase();
            m_actualMove = 1; // for target completion of movement
        }
    }
    else if (m_speed < -math::epsilonFloat && !m_isBlocked)
    {
        m_actualMove--;

        if (m_actualMove == 0)
        {
            switchPhase();
            m_actualMove = m_countMoves;
        }
    }
}

void GaitHandler::switchPhase()
{
    m_actualStep++;
    switch (m_gaitNr)
    {
    default:
    case gaits::type::target:
        shiftArrayRight<uint8_t>(m_actualLegStates, msrh01::legs);
        if (m_actualStep == msrh01::legs) // six legs moved to target location
        {
            finished();
            if (m_gaitNrAfterTarget != m_gaitNr)
            {
                setInternalGait(m_gaitNrAfterTarget);
            }
            else
            {
                for (uint8_t i = 0; i < msrh01::legs; ++i)
                {
                    m_actualLegStates[i] = gaits::state::stop;
                }
            }
        }
        break;
    case gaits::type::tripod:
        for (uint8_t i = 0; i < msrh01::legs; ++i)
        {
            switch (m_actualLegStates[i])
            {
            case gaits::state::swing:
                m_actualLegStates[i] = gaits::state::stance;
                break;
            case gaits::state::stance:
                m_actualLegStates[i] = gaits::state::swing;
                break;
            case gaits::state::stop:
            default:
                break;
            }
        }
        break;
    case gaits::type::wave:
        shiftArrayRight<uint8_t>(m_actualLegStates, msrh01::legs);
        break;
    }
}

void GaitHandler::setGait(uint8_t gaitNr)
{
    m_kinematicControl.calcInterpolationToStart(gaitNr);
    m_gaitNrAfterTarget = gaitNr;
    setInternalGait(gaits::target);
}

void GaitHandler::setInternalGait(uint8_t gaitNr)
{
    m_gaitNr = gaitNr;
    setStartStates();
    setActualMove(0);
    setSpeed(0.0f);
    m_actualStep = 0;
}
