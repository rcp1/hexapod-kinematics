#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#include "trafoBodyToJointFrame.hpp"
#include "pose3d.h"
#include "defines.h"

class KinematicModel
{
public:
    KinematicModel();

    virtual ~KinematicModel();

    Vector3d getInitPosition(uint8_t legIndex) const;

    Vector3d getActualPosition(uint8_t legIndex) const;

    trafoStatus calcJointAngles(Vector3d foot, const Pose3d& body, const uint8_t legIndex, Vector3d& output);

protected:
    Vector3d calcYawedCurve(const Vector3d& foot, const uint8_t legIndex) const;

    Vector3d m_initFeet[msrh01::legs];

    Vector3d m_actualFoot[msrh01::legs];

    static TrafoBodyToJointFrame<msrh01::fr> s_bodyToJointFrontRight;

    static TrafoBodyToJointFrame<msrh01::fl> s_bodyToJointFrontLeft;

    static TrafoBodyToJointFrame<msrh01::mr> s_bodyToJointMidRight;

    static TrafoBodyToJointFrame<msrh01::ml> s_bodyToJointMidLeft;

    static TrafoBodyToJointFrame<msrh01::br> s_bodyToJointBackRight;

    static TrafoBodyToJointFrame<msrh01::bl> s_bodyToJointBackLeft;

    static const TrafoBase* legTrafos[msrh01::legs];

};

inline
Vector3d KinematicModel::getInitPosition(uint8_t legIndex) const
{
    return m_initFeet[legIndex];
}

inline
Vector3d KinematicModel::getActualPosition(uint8_t legIndex) const
{
    return m_actualFoot[legIndex];
}

#endif // KINEMATICS_H
