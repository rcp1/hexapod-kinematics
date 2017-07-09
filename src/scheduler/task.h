#ifndef TASK_H
#define TASK_H

#include "defines.h"
#include "interrupt.h"

#ifdef ARDUINO_PLATFORM
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>
#endif

class Task
{
public:
    Task(uint32_t interval);
    virtual ~Task();
    bool isActive();
    void run();
    void finished();

private:
    bool m_isActive;

    // interrupt handler
    class taskInterrupt : public Interrupt
    {
    public:
        taskInterrupt(Task* ownerTask, uint32_t interval) : m_ownerTask(ownerTask), m_timerNumber(0)
        {
            initInterruptTimer(interval);
            record(m_timerNumber, this);
        }
        Task* m_ownerTask;
        void serviceRoutine();
        void initInterruptTimer(uint32_t interval);
    private:
        uint8_t m_timerNumber;
        static bool m_isTimerUsed[4];

    }m_nestedTaskInterrupt;

    friend taskInterrupt;
};

#endif // TASK_H
