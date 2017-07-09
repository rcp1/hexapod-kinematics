#ifndef KINEMATICCONTROL_H
#define KINEMATICCONTROL_H

#include "defines.h"
#include "trafoCoordLegToLeg.h"
#include "trafoCoordBodyToLeg.h"
#include "trafoCoordBodyToHip.h"
#include "pose3d.h"
#include "vector3d.h"
#include "servoTypes.h"
#include "interpolationHandler.h"

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

    void calcInterpolationBody(const Pose3d& MCSDesiredBodyPose);

    void calcKin();

    void setServos();

    float calcSwingTime(const float& speed) const;

    inline float getSwingLength() { return m_swingLength; };

    inline Pose3d getBodyPose() { return m_MCSBodyPose; };

private:
    Vector3d calcKinLeg(const Vector3d& BCSTCPStart, const Pose3d& MCSBody, const uint8_t legIndex);

    Vector3d calcYawedCurve(const Vector3d& MCSTCPInit, const Vector3d& BCSTCPBezier) const;

    void initLegServoVectors();

    float calcTurnDistance(const float& turnAngle);

    float calcMaxTurnRadiusOfLegs(const float& turnDistance);

    float calcArcAngle(const float& radiusOfTurn);

    InterpolationHandler& m_interpolation;
    GaitHandler& m_gait;
    BodyHandler& m_body_pst;
    LegServoVector m_legServoVectors_ast[msrh01::legs];
    Vector3d m_BCSTCPActualGait[msrh01::legs];

    TrafoCoordBodyToLeg m_trafoCoordBodyToLeg;
    TrafoCoordBodyToHip m_trafoCoordBodyToHip;
    TrafoCoordLegToLeg m_trafoCoordLegToLeg;
    TrafoKin3AxisLeg m_trafoKin3AxisLeg;

    Pose3d m_MCSBodyPose;
    Vector3d m_BCSTCPInit;

    float m_arcAngle; // rad
    float m_swingLength; // %
    float m_turnDistance; // m
};

#endif // KINEMATICCONTROL_H
