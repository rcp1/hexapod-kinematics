#ifndef  BODYHANDLER_H
#define  BODYHANDLER_H

#include "defines.h"
#include "moveHandler.h"

class BodyHandler : public MoveHandler
{
public:
    BodyHandler(KinematicControl& kinematicControl, uint32_t interval);

    virtual ~BodyHandler();

    void transition();
};

#endif // BODYHANDLER_H
