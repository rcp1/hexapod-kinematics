#include "interpolationHandler.h"

BezierApproximation<Vector3d, InterpolationHandler::bezierTargetAnchors> InterpolationHandler::m_bezierTargetSwing[msrh01::legs];
BezierApproximation<Vector3d, InterpolationHandler::bezierStanceAnchors> InterpolationHandler::m_bezierStance;
BezierApproximation<Vector3d, InterpolationHandler::bezierSwingAnchors> InterpolationHandler::m_bezierSwing;
BezierApproximation<Vector3d, InterpolationHandler::bezierSwingAnchors> InterpolationHandler::m_actualInterpolation[msrh01::legs];
BezierApproximation<Vector3d, InterpolationHandler::bezierBodyAnchors> InterpolationHandler::m_bezierBodyPos;
BezierApproximation<Orientation3d, InterpolationHandler::bezierBodyAnchors> InterpolationHandler::m_bezierBodyOri;

InterpolationHandler::InterpolationHandler()
{
    initBezier();
}

InterpolationHandler::~InterpolationHandler()
{
}

void InterpolationHandler::initBezier()
{
    // Bezier curve with -1 to 1 in x and 0 to 1 in z
    const Vector3d b1 = Vector3d(-0.83775f, 0.f, 0.f);
    const Vector3d b2 = Vector3d(-1.11701f, 0.f, 0.f);
    const Vector3d b3 = Vector3d(-1.39626f, 0.f, 0.f);
    const Vector3d b4 = Vector3d(0.f, 0.f, 3.2f);
    const Vector3d b5 = Vector3d(1.39626f, 0.f, 0.f);
    const Vector3d b6 = Vector3d(1.11701f, 0.f, 0.f);
    const Vector3d b7 = Vector3d(0.83775f, 0.f, 0.f);

    m_bezierSwing.addAnchor(b1);
    m_bezierSwing.addAnchor(b2);
    m_bezierSwing.addAnchor(b3);
    m_bezierSwing.addAnchor(b4);
    m_bezierSwing.addAnchor(b5);
    m_bezierSwing.addAnchor(b6);
    m_bezierSwing.addAnchor(b7);

    const Vector3d b21 = Vector3d(0.83775f, 0.f, 0.f);
    const Vector3d b22 = Vector3d(-0.83775f, 0.f, 0.f);

    m_bezierStance.addAnchor(b21);
    m_bezierStance.addAnchor(b22);
}

void InterpolationHandler::calcInterpolationToTarget(const uint8_t& legIndex, const Vector3d& BCSFrom, const Vector3d& BCSTarget, const float& height)
{
    m_bezierTargetSwing[legIndex].reset();
    m_bezierTargetSwing[legIndex].addAnchor(BCSFrom);
    Vector3d middleVector = (BCSTarget + BCSFrom) / 2.f;
    middleVector.z += height;
    m_bezierTargetSwing[legIndex].addAnchor(middleVector);
    m_bezierTargetSwing[legIndex].addAnchor(BCSTarget);
}

void InterpolationHandler::calcInterpolationBody(const Pose3d& MCSFrom, const Pose3d& MCSTarget)
{
	m_bezierBodyPos.reset();
	m_bezierBodyPos.addAnchor(MCSFrom.m_position);
	m_bezierBodyPos.addAnchor(MCSTarget.m_position);

	m_bezierBodyOri.reset();
	m_bezierBodyOri.addAnchor(MCSFrom.m_orientation);
	m_bezierBodyOri.addAnchor(MCSTarget.m_orientation);
}

Vector3d InterpolationHandler::getGaitInterpolation(const uint8_t& state, const uint8_t& legIndex, const float& time)
{
    switch (state)
    {
    default:
    case gaits::state::targetSwing:
        return m_bezierTargetSwing[legIndex].getPos(time);
    case gaits::state::stance:
        return m_bezierStance.getPos(time);
    case gaits::state::swing:
        return m_bezierSwing.getPos(time);
    }
}

Pose3d InterpolationHandler::getBodyInterpolation(const float& time)
{
    return Pose3d(m_bezierBodyPos.getPos(time), m_bezierBodyOri.getPos(time));
}

