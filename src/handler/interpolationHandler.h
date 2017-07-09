#ifndef INTERPOLATIONHANDLER_H
#define INTERPOLATIONHANDLER_H

#include "pose3d.h"
#include "vector3d.h"
#include "orientation3d.h"
#include "bezierApproximation.h"
#include "defines.h"

class InterpolationHandler
{
public:
    InterpolationHandler();

    virtual ~InterpolationHandler();

    void calcInterpolationToTarget(const uint8_t& legIndex, const Vector3d& BCSFrom, const Vector3d& BCSTarget, const float& height = 0.f);

    void calcInterpolationBody(const Pose3d& BCSFrom, const Pose3d& BCSTarget);

    Vector3d getGaitInterpolation(const uint8_t& state, const uint8_t& legIndex, const float& time);

    Pose3d getBodyInterpolation(const float& time);

protected:
    void initBezier();

    static const uint8_t bezierSwingAnchors = (uint8_t)7;
    static const uint8_t bezierStanceAnchors = (uint8_t)2;
    static const uint8_t bezierTargetAnchors = (uint8_t)3;
    static const uint8_t bezierBodyAnchors = (uint8_t)2;

    static BezierApproximation<Vector3d, bezierTargetAnchors> m_bezierTargetSwing[msrh01::legs];
    static BezierApproximation<Vector3d, bezierStanceAnchors> m_bezierStance;
    static BezierApproximation<Vector3d, bezierSwingAnchors> m_bezierSwing;
    static BezierApproximation<Vector3d, bezierSwingAnchors> m_actualInterpolation[msrh01::legs];
    static BezierApproximation<Vector3d, bezierBodyAnchors> m_bezierBodyPos; ///> 1 for x,y,z; 2 for a,b,c
    static BezierApproximation<Orientation3d, bezierBodyAnchors> m_bezierBodyOri; ///> 1 for x,y,z; 2 for a,b,c
};

#endif
