#ifndef GAITHANDLER_H
#define GAITHANDLER_H

#include "helper.h"
#include "defines.h"
#include "moveHandler.h"

class GaitHandler : public MoveHandler
{
public:
    GaitHandler(KinematicControl& kinematicControl, uint32_t interval, uint8_t gaitNr = gaits::type::tripod);

    void setStartStates();

    uint8_t getStartStates(uint8_t gaitNr, uint8_t legIndex);

    virtual ~GaitHandler();

    void transition();

    void setGait(uint8_t gaitNr);

    inline uint8_t getGait()
    {
        return m_gaitNr;
    };

    inline uint8_t getLegState(const uint8_t& index)
    {
        return (m_actualLegStates[index]);
    };

    inline void setLegState(const uint8_t& index, const uint8_t& legState)
    {
        m_actualLegStates[index] = legState;
    };

private:
    void switchPhase();

    void setInternalGait(uint8_t gaitNr);

    uint8_t m_gaitNr;
    uint8_t m_gaitNrAfterTarget;
    uint32_t m_actualStep;

    uint8_t m_actualLegStates[msrh01::legs];
};

#endif // GAITHANDLER_H
