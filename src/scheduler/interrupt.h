#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "defines.h"

class Interrupt
{
public:
    static Interrupt* m_owners[4];

    virtual void serviceRoutine() = 0;
    static void handler1();
    static void handler2();
    static void handler3();
    static void handler4();
    static void record(uint8_t m_timerNumber, Interrupt *i);
};

#endif // INTERRUPT_H
