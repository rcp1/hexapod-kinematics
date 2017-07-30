#include "kinematicControl.h"
#include "gaitHandler.h"
#include "bodyHandler.h"
#include "mathConstants.h"


KinematicControl::KinematicControl(InterpolationHandler& interpolationHandler, GaitHandler& gaitHandler_pst, BodyHandler& bodyHandler_pst) :
    m_kinematic(),
    m_interpolation(interpolationHandler),
    m_gait(gaitHandler_pst),
    m_body_pst(bodyHandler_pst),
    m_arcAngle(0.0f),
    m_swingLength(1.0f),
    m_turnDistance(0.0f)
{
    initLegServoVectors();
    setTurnAngle(0.0f);
}

void KinematicControl::initLegServoVectors()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_legServoVectors_ast[i].setLegIndex(i);
        m_legServoVectors_ast[i].setMoveTime(tasks::servoInterval);
        m_legServoVectors_ast[i].init();
    }
}

KinematicControl::~KinematicControl()
{
}

void KinematicControl::setServoMoveTime(uint16_t moveTime)
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_legServoVectors_ast[i].setMoveTime(moveTime);
    }
}

void KinematicControl::setTurnAngle(const float& turnAngle)
{
    m_turnDistance = calcTurnDistance(turnAngle);

    const float maxRadiusOfTurn = calcMaxTurnRadiusOfLegs(m_turnDistance);
    m_arcAngle = calcArcAngle(maxRadiusOfTurn);
}

void KinematicControl::setSwingLength(const float& swingLength)
{
    m_swingLength = swingLength;
    const float maxRadiusOfTurn = calcMaxTurnRadiusOfLegs(m_turnDistance);
    m_arcAngle = calcArcAngle(maxRadiusOfTurn);
}

void KinematicControl::setNewBodyPose(const Pose3d& bodyPose)
{
    m_body_pst.setActualMove(0);
    if (!m_body_pst.isBlocked())
    {
        calcInterpolationBody(bodyPose);
    }
    m_bodyPose = bodyPose;
}

void KinematicControl::setInitialBodyPose(const Pose3d& bodyPose)
{
    m_body_pst.setActualMove(0);
    m_bodyPose = bodyPose;
    calcInterpolationBody(bodyPose);
}

void KinematicControl::calcInterpolationToStart(uint8_t gaitNr)
{
    Vector3d target;
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        target = m_interpolation.getGaitInterpolation(m_gait.getStartStates(gaitNr, i), i, 0.f);
        target.x *= m_swingLength * msrh01::stepLengthMaximum;
        target += m_kinematic.getInitPosition(i);
        m_interpolation.calcInterpolationToTarget(i, m_kinematic.getActualPosition(i), target, m_swingLength * msrh01::stepLengthMaximum);
    }
}

void KinematicControl::calcInterpolationToInit()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_interpolation.calcInterpolationToTarget(i, m_kinematic.getActualPosition(i), m_kinematic.getInitPosition(i), m_swingLength * msrh01::stepLengthMaximum);
    }
}

void KinematicControl::calcInterpolationBody(const Pose3d& bodyPose)
{
    m_interpolation.calcInterpolationBody(m_bodyPose, bodyPose);
}

void KinematicControl::calcKin()
{
    Vector3d bezierFoot; ///> foot position in body frame
    Vector3d footJoint; ///> foot position in joint frame (angles)

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        if (m_gait.getLegState(i) != gaits::state::stop)
        {
            // get actual bezier foot position in body frame
            bezierFoot = m_interpolation.getGaitInterpolation(m_gait.getLegState(i), i, m_gait.getActualTime());
            if (m_gait.getLegState(i) != gaits::state::targetSwing)
            {
                bezierFoot = calcYawedCurve(bezierFoot, i);
            }
        }
        else
        {
            bezierFoot = m_kinematic.getActualPosition(i);
        }
        m_bodyPose = m_interpolation.getBodyInterpolation(m_body_pst.getActualTime());

        trafoStatus result = m_kinematic.calcJointAngles(bezierFoot, m_bodyPose, i, footJoint);

        m_legServoVectors_ast[i].setTrafoVector(footJoint);
        // set new ACS TCP
    }
}

Vector3d KinematicControl::calcYawedCurve(const Vector3d& foot, const uint8_t legIndex) const
{
    Vector3d yawedFoot;
    yawedFoot = m_kinematic.getInitPosition(legIndex);

    // translate to center of rotation
    yawedFoot.y -= m_turnDistance;

    // cart to polar
    const float r = sqrtf(pow(yawedFoot.x, 2) + pow(yawedFoot.y, 2));
    const float phi = atan2f(yawedFoot.y, yawedFoot.x) + foot.x * m_arcAngle;

    // pol to cart
    yawedFoot.x = r * cosf(phi);
    yawedFoot.y = r * sinf(phi);

    // translate back to center of robot
    yawedFoot.y += m_turnDistance;
    yawedFoot.z = m_kinematic.getInitPosition(legIndex).z + m_swingLength * msrh01::stepLengthMaximum * foot.z;


    return yawedFoot;
}

void KinematicControl::setServos()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_legServoVectors_ast[i].setServos();
    }
}

float KinematicControl::calcTurnDistance(const float& turnAngle)
{
    float turnDistance = 0.f;

    if (fabs((turnAngle + 1.f)) < math::epsilonFloat)
    {
        turnDistance = tanf((0.f - math::epsilonFloat) * M_PI_2);
    }
    else if (fabs((turnAngle - 1.f)) < math::epsilonFloat)
    {
        turnDistance = tanf((0.f + math::epsilonFloat) * M_PI_2);
    }
    else if (abs(turnAngle) <= toleranceTurn)
    {
        turnDistance = tanf((1.f - toleranceTurn) * M_PI_2);
    }
    else if (fabs(turnAngle) > toleranceTurn)
    {
        turnDistance = tanf((1.f - turnAngle) * M_PI_2);
    }

    return turnDistance;
}

float KinematicControl::calcMaxTurnRadiusOfLegs(const float &turnDistance)
{
    float maxRadiusOfTurn = 0.0f;
    Vector3d init;

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        init = m_kinematic.getInitPosition(i);
        init.y -= turnDistance;
        // cart to polar
        const float r = sqrtf(pow(init.x, 2) + pow(init.y, 2));
        if (r >= maxRadiusOfTurn)
        {
            maxRadiusOfTurn = r;
        }
    }
    if (turnDistance < 0.f)
        maxRadiusOfTurn = -maxRadiusOfTurn;

    return maxRadiusOfTurn;
}

float KinematicControl::calcArcAngle(const float& radiusOfTurn)
{
    return (m_swingLength * msrh01::stepLengthMaximum) / radiusOfTurn;
}

float KinematicControl::calcSwingTime(const float& speed) const
{
    if ((float)fabs(speed) > math::epsilonFloat)
    {
        // m_swing [m] / (v_act [m/s] / 1000) [m/ms]
        return maximum(((m_swingLength * msrh01::stepLengthMaximum) / ((float)fabs(speed) / 1000.0f)), 300.0f);
    }
    else
    {
        return 65535;
    }
}
