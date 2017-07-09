#include <interrupt.h>

Interrupt* Interrupt::m_owners[] = { 0 };

Interrupt::Interrupt()
{
}

Interrupt::~Interrupt()
{
}

void Interrupt::handler1()
{
    if(m_owners[0])
    {
        m_owners[0]->serviceRoutine();
    }
}
void Interrupt::handler2()
{
    if (m_owners[1])
    {
        m_owners[1]->serviceRoutine();
    }
}
void Interrupt::handler3()
{
    if (m_owners[2])
    {
        m_owners[2]->serviceRoutine();
    }
}
void Interrupt::handler4()
{
    if (m_owners[3])
    {
        m_owners[3]->serviceRoutine();
    }
}

void Interrupt::record(uint8_t timerNumber, Interrupt *i)
{
    m_owners[timerNumber] = i;
}
