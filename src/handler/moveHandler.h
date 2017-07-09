#ifndef MOVEHANDLER_H
#define MOVEHANDLER_H

#include "kinematicControl.h"
#include "defines.h"

class KinematicControl;

class MoveHandler
{
public:
    MoveHandler(KinematicControl& kinematicControl, uint32_t interval);

    virtual ~MoveHandler();

    virtual void transition() = 0;

    void setSpeed(const float& speed);

    inline uint8_t getCountOfMoves()
    {
        return m_countMoves;
    };

    void setActualMove(const uint8_t& actualMove)
    {
        if (!m_isBlocked)
        {
            m_isFinished = false;
            m_actualMove = actualMove;
        }
    };

    uint8_t getActualMove()
    {
        return m_actualMove;
    };

    inline float getActualTime()
    {
        return (float)((float)m_actualMove / (float)m_countMoves);
    };

    inline void setBlocking(bool block)
    {
        m_isBlocked = block;
    }

    inline bool isBlocked()
    {
        return m_isBlocked;
    }

    inline float getSpeed()
    {
        return m_speed;
    }

    inline void finished()
    {
        m_isFinished = true;
    }

    inline bool getFinished()
    {
        return m_isFinished;
    }

protected:
    void calcCountOfMoves(const float& swingTime);

    KinematicControl& m_kinematicControl;

    uint32_t m_interval; ///> [ms]
    uint8_t m_countMoves;
    uint8_t m_actualMove;
    float m_speed; ///> [m/s]
    bool m_isBlocked;
    bool m_isFinished;
};

#endif // MOVEHANDLER_H
