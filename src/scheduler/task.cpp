#include "task.h"

bool Task::taskInterrupt::m_isTimerUsed[] = { false };

Task::Task(uint32_t interval) :
	m_nestedTaskInterrupt(this, interval)
{
}

Task::~Task()
{
}

bool Task::isActive()
{
    return m_isActive;
}

void Task::run()
{
    m_isActive = true;
}

void Task::finished()
{
    m_isActive = false;
}

void Task::taskInterrupt::serviceRoutine()
{
    m_ownerTask->run();
}

void Task::taskInterrupt::initInterruptTimer(uint32_t interval)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!m_isTimerUsed[i])
        {
            m_timerNumber = i;
            m_isTimerUsed[i] = true;
            break;
        }
    }

    switch (m_timerNumber)
    {
    case 0:
        Timer1.initialize(interval * 1000UL); // needs microseconds
        Timer1.attachInterrupt(handler1);
        break;
    case 1:
        Timer3.initialize(interval * 1000UL); // needs microseconds
        Timer3.attachInterrupt(handler2);
        break;
    case 2:
        Timer4.initialize(interval * 1000UL); // needs microseconds
        Timer4.attachInterrupt(handler3);
        break;
    case 3:
        Timer5.initialize(interval * 1000UL); // needs microseconds
        Timer5.attachInterrupt(handler4);
        break;
    }
}
