#include "bodyHandler.h"

BodyHandler::BodyHandler(KinematicControl& kinematicControl, uint32_t interval) :
    MoveHandler(kinematicControl, interval)
{
}

BodyHandler::~BodyHandler()
{
}

void BodyHandler::transition()
{
    if (m_actualMove < m_countMoves)
    {
        m_actualMove++;
    }
    else
    {
        finished();
    }
}
