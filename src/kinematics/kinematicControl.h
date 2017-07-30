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

    void setCurvature(float curvature);

    void setSwingLength(float swingLength);

    void setMoveAngle(float moveAngle);

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

    Vector3d calcMoveDirection(const Vector3d& foot, const uint8_t legIndex) const;

    void calcTurnRadius(float curvature);

    float calcMaxTurnRadiusOfLegs();

    KinematicModel m_kinematic;

    InterpolationHandler& m_interpolation;

    GaitHandler& m_gait;

    BodyHandler& m_body_pst;

    LegServoVector m_legServoVectors_ast[msrh01::legs];

    Pose3d m_bodyPose;

    float m_maxRadiusOfTurn; // m

    float m_moveAngle; // rad

    float m_swingLength; // %

    float m_turnRadius; // m
};

#endif // KINEMATICCONTROL_H
