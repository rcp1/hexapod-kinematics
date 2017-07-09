#include "kinematicControl.h"
#include "gaitHandler.h"
#include "bodyHandler.h"
#include "mathConstants.h"

KinematicControl::KinematicControl(InterpolationHandler& interpolationHandler, GaitHandler& gaitHandler_pst, BodyHandler& bodyHandler_pst) :
    m_interpolation(interpolationHandler),
    m_gait(gaitHandler_pst),
    m_body_pst(bodyHandler_pst),
    m_arcAngle(0.0f),
    m_swingLength(1.0f),
    m_turnDistance(0.0f)
{
    m_trafoKin3AxisLeg.forward(Vector3d(msrh01::anglesInit[msrh01::coxaIndex],
                                              msrh01::anglesInit[msrh01::femurIndex],
                                              msrh01::anglesInit[msrh01::tibiaIndex]), m_BCSTCPInit);
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
        m_BCSTCPActualGait[i] = m_BCSTCPInit;
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
    m_MCSBodyPose = bodyPose;
}

void KinematicControl::setInitialBodyPose(const Pose3d& bodyPose)
{
    m_body_pst.setActualMove(0);
    m_MCSBodyPose = bodyPose;
    calcInterpolationBody(bodyPose);
}

void KinematicControl::calcInterpolationToStart(uint8_t gaitNr)
{
    Vector3d BCSTCPTarget;
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_trafoCoordLegToLeg.forward(m_BCSTCPInit + m_interpolation.getGaitInterpolation(m_gait.getStartStates(gaitNr, i), i, 0.0f) * m_swingLength * msrh01::stepLengthMaximum, BCSTCPTarget, i);
        m_trafoCoordBodyToHip.forward(BCSTCPTarget, BCSTCPTarget, i);
        m_interpolation.calcInterpolationToTarget(i, m_BCSTCPActualGait[i], BCSTCPTarget, m_swingLength * msrh01::stepLengthMaximum);
    }
}

void KinematicControl::calcInterpolationToInit()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_interpolation.calcInterpolationToTarget(i, m_BCSTCPActualGait[i], m_BCSTCPInit, m_swingLength * msrh01::stepLengthMaximum);
    }
}

void KinematicControl::calcInterpolationBody(const Pose3d& MCSDesiredBodyPose)
{
    m_interpolation.calcInterpolationBody(m_MCSBodyPose, MCSDesiredBodyPose);
}

void KinematicControl::calcKin()
{
    Vector3d ACSTCP;
    Vector3d BCSTCPBezier;
    Vector3d MCSBody;

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        if (m_gait.getLegState(i) != gaits::state::stop)
        {
            // get actual bezier TCP in BCS of middle right leg
            BCSTCPBezier = m_interpolation.getGaitInterpolation(m_gait.getLegState(i), i, m_gait.getActualTime());
        }
        else
        {
            BCSTCPBezier = m_BCSTCPActualGait[i];
        }
        m_MCSBodyPose = m_interpolation.getBodyInterpolation(m_body_pst.getActualTime());

        ACSTCP = calcKinLeg(BCSTCPBezier, m_MCSBodyPose, i);
        // set new ACS TCP
        m_legServoVectors_ast[i].setTrafoVector(ACSTCP);
    }
}

Vector3d KinematicControl::calcKinLeg(const Vector3d& BCSTCPStart, const Pose3d& MCSBody, const uint8_t legIndex)
{
    Vector3d MCSTCPInit;
    Vector3d MCSTCPnew;
    Vector3d BCSTCPBezier;
    Vector3d ACSTCP;

    Pose3d BCSTCPBezierAndBody;

    BCSTCPBezier = BCSTCPStart;

    // get init position of leg in MCS
    m_trafoCoordLegToLeg.forward(m_BCSTCPInit, MCSTCPInit, legIndex);

    if (m_gait.getGait() != gaits::type::target)
    {
        MCSTCPnew = calcYawedCurve(MCSTCPInit, BCSTCPBezier);

        // transform yawed Bezier MCS TCP to BCS
        m_trafoCoordBodyToHip.forward(MCSTCPnew, BCSTCPBezier, legIndex);
    }
    m_BCSTCPActualGait[legIndex] = BCSTCPBezier;
    // calculate desired body position with actual Bezier TCP
    m_trafoCoordBodyToLeg.forward(MCSBody, BCSTCPBezier, BCSTCPBezierAndBody, legIndex);

    // get TCP in ACS
    trafoStatus result = m_trafoKin3AxisLeg.backward(BCSTCPBezierAndBody.m_position, ACSTCP);
    if (result != trafoOk)
    {
        Serial.print("Trafo Error: "); Serial.println(result);
        ACSTCP = m_legServoVectors_ast[legIndex].getTrafoVector();
    }

    return ACSTCP;
}

Vector3d KinematicControl::calcYawedCurve(const Vector3d &MCSTCPInit, const Vector3d &BCSTCPBezier) const
{
    Vector3d MCSTCPnew;;
    // calculated yawed Bezier curve with arc angle and lateral turn distance
    MCSTCPnew = MCSTCPInit;
    MCSTCPnew.y -= m_turnDistance;

    // cart to polar
    float r = MCSTCPnew.length();
    float phi = atan2f(MCSTCPnew.y, MCSTCPnew.x);
    phi += BCSTCPBezier.x * m_arcAngle;
    // pol to cart
    MCSTCPnew.x = r * cosf(phi);
    MCSTCPnew.y = r * sinf(phi);

    MCSTCPnew.y += m_turnDistance;
    MCSTCPnew.z = MCSTCPInit.z + m_swingLength * msrh01::stepLengthMaximum * BCSTCPBezier.z;

    return MCSTCPnew;
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
    float turnDistance = 0.0f;

    if ((float)fabs((turnAngle + 1.0f)) < math::epsilonFloat)
    {
        turnDistance = (float)tanf((0.0f - math::epsilonFloat) * M_PI_2);
    }
    else if ((float)fabs((turnAngle - 1.0f)) < math::epsilonFloat)
    {
        turnDistance = (float)tanf((0.0f + math::epsilonFloat) * M_PI_2);
    }
    else if ((float)abs(turnAngle) <= toleranceTurn)
    {
        turnDistance = (float)tanf((1.0f - toleranceTurn) * M_PI_2);
    }
    else if ((float)fabs(turnAngle) > toleranceTurn)
    {
        turnDistance = (float)tanf((1.0f - turnAngle) * M_PI_2);
    }

    return turnDistance;
}

float KinematicControl::calcMaxTurnRadiusOfLegs(const float &turnDistance)
{
    float maxRadiusOfTurn = 0.0f;
    Vector3d MCSInit;

    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        m_trafoCoordBodyToHip.backward(m_BCSTCPInit, MCSInit, i);
        MCSInit.y -= turnDistance;
        // cart to polar
        float r = MCSInit.length();
        if (r >= maxRadiusOfTurn)
        {
            maxRadiusOfTurn = r;
        }
    }
    if (turnDistance < 0.0f)
        maxRadiusOfTurn = -maxRadiusOfTurn;

    return maxRadiusOfTurn;
}

float KinematicControl::calcArcAngle(const float &radiusOfTurn)
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
