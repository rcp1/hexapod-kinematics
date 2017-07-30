#include "kinematicControl.h"
#include "gaitHandler.h"
#include "bodyHandler.h"
#include "mathConstants.h"


KinematicControl::KinematicControl(InterpolationHandler& interpolationHandler, GaitHandler& gaitHandler_pst, BodyHandler& bodyHandler_pst) :
    m_kinematic(),
    m_interpolation(interpolationHandler),
    m_gait(gaitHandler_pst),
    m_body_pst(bodyHandler_pst),
    m_maxRadiusOfTurn(0.0f),
    m_moveAngle(0.0f),
    m_swingLength(1.0f),
    m_turnRadius(0.0f)
{
    initLegServoVectors();
    setCurvature(0.0f);
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

void KinematicControl::setCurvature(float curvature)
{
    calcTurnRadius(curvature);
    m_maxRadiusOfTurn = calcMaxTurnRadiusOfLegs();
}

void KinematicControl::setSwingLength(float swingLength)
{
    m_swingLength = swingLength;
    m_maxRadiusOfTurn = calcMaxTurnRadiusOfLegs();
}

void KinematicControl::setMoveAngle(float moveAngle)
{
    m_moveAngle = moveAngle;
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
    Vector3d foot; ///> foot position in body frame
    Vector3d footJoint; ///> foot position in joint frame (angles)

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        if (m_gait.getLegState(i) == gaits::state::stop)
        {
            foot = m_kinematic.getActualPosition(i);
        }
        else
        {
            // get actual bezier foot position in body frame
            const Vector3d bezierFoot = m_interpolation.getGaitInterpolation(m_gait.getLegState(i), i, m_gait.getActualTime());

            if (m_gait.getLegState(i) == gaits::state::targetSwing)
            {
                foot = bezierFoot;
            }
            else
            {
                foot = calcMoveDirection(bezierFoot, i);
            }
        }

        // get actual body pose in body frame
        m_bodyPose = m_interpolation.getBodyInterpolation(m_body_pst.getActualTime());

        trafoStatus trafoStatus = m_kinematic.calcJointAngles(foot, m_bodyPose, i, footJoint);
        if (trafoStatus == trafoOk)
        {
            // set new foot position in joint frame
            m_legServoVectors_ast[i].setTrafoVector(footJoint);
        }

    }
}

Vector3d KinematicControl::calcMoveDirection(const Vector3d& bezierFoot, const uint8_t legIndex) const
{
    // scale Bezier curve with step length
    const Vector3d scaledFoot = bezierFoot * m_swingLength * msrh01::stepLengthMaximum;
    // rotate center of rotation to desired move direction
    const Vector3d toRotationFrame = math::rotateZ(Vector3d(0.f, m_turnRadius, 0.f), m_moveAngle);

    Vector3d foot = m_kinematic.getInitPosition(legIndex);

    // translate to frame of rotation
    foot -= toRotationFrame;

    // cartesian to polar
    const float r = sqrtf(pow(foot.x, 2) + pow(foot.y, 2));
    // add yaw angle to phi
    const float phi = atan2f(foot.y, foot.x) + scaledFoot.x / m_maxRadiusOfTurn;

    // polar to cartesian
    foot.x = r * cosf(phi);
    foot.y = r * sinf(phi);

    // translate back to frame of body
    foot += toRotationFrame;
    foot.z = m_kinematic.getInitPosition(legIndex).z + scaledFoot.z;

    return foot;
}

void KinematicControl::setServos()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_legServoVectors_ast[i].setServos();
    }
}

void KinematicControl::calcTurnRadius(float curvature)
{
    if (fabs(curvature) > curvatureTolerance)
    {
        m_turnRadius = 1.f / curvature;
    }
    else
    {
        // straight walking
        m_turnRadius = 10000.f;
    }
}

float KinematicControl::calcMaxTurnRadiusOfLegs()
{
    float maxLegTurnRadius = 0.f;
    Vector3d init;

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        init = m_kinematic.getInitPosition(i);
        init.y -= m_turnRadius;

        // cart to polar
        const float r = sqrtf(pow(init.x, 2) + pow(init.y, 2));

        if (r >= maxLegTurnRadius)
        {
            maxLegTurnRadius = r;
        }
    }
    if (m_turnRadius < 0.f)
        maxLegTurnRadius = -maxLegTurnRadius;

    return maxLegTurnRadius;
}

float KinematicControl::calcSwingTime(const float& speed) const
{
    if ((float)fabs(speed) > math::epsilonFloat)
    {
        // m_swing [m] / (v_act [m/s] / 1000) [m/ms]
        return maximum(((m_swingLength * msrh01::stepLengthMaximum) / ((float)fabs(speed) / 1000.f)), 300.f);
    }
    else
    {
        return 65535;
    }
}
