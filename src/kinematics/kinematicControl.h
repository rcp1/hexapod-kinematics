#ifndef KINEMATICCONTROL_H
#define KINEMATICCONTROL_H

#include "defines.h"
#include "pose3d.h"
#include "vector3d.h"
#include "servoTypes.h"
#include "interpolationHandler.h"
#include "kinematicModel.h"

class BodyHandler;
class GaitHandler;

class KinematicControl
{
public:
    KinematicControl(InterpolationHandler& interpolationHandler, GaitHandler& gaitHandler, BodyHandler& bodyHandler);

    virtual ~KinematicControl();

    void setServoMoveTime(uint16_t moveTime);

    void setTurnAngle(const float& turnAngle);

    void setSwingLength(const float& swingLength);

    void setNewBodyPose(const Pose3d& bodyPose);

    void setInitialBodyPose(const Pose3d& bodyPose);

    void calcInterpolationToStart(uint8_t gaitNr);

    void calcInterpolationToInit();

    void calcInterpolationBody(const Pose3d& bodyPose);

    void calcKin();

    void setServos();

    float calcSwingTime(const float& speed) const;

    inline float getSwingLength() { return m_swingLength; };

    inline Pose3d getBodyPose() { return m_bodyPose; };

private:

    void initLegServoVectors();

    Vector3d calcYawedCurve(const Vector3d& foot, const uint8_t legIndex) const;

    float calcTurnDistance(const float& turnAngle);

    float calcMaxTurnRadiusOfLegs(const float& turnDistance);

    float calcArcAngle(const float& radiusOfTurn);

    KinematicModel m_kinematic;

    InterpolationHandler& m_interpolation;

    GaitHandler& m_gait;

    BodyHandler& m_body_pst;

    LegServoVector m_legServoVectors_ast[msrh01::legs];

    Pose3d m_bodyPose;

    float m_arcAngle; // rad

    float m_swingLength; // %

    float m_turnDistance; // m
};

#endif // KINEMATICCONTROL_H
