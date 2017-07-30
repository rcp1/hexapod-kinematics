#include "kinematicModel.h"
#include "rotate.h"

TrafoBodyToJointFrame<msrh01::fr> KinematicModel::s_bodyToJointFrontRight;
TrafoBodyToJointFrame<msrh01::fl> KinematicModel::s_bodyToJointFrontLeft;
TrafoBodyToJointFrame<msrh01::mr> KinematicModel::s_bodyToJointMidRight;
TrafoBodyToJointFrame<msrh01::ml> KinematicModel::s_bodyToJointMidLeft;
TrafoBodyToJointFrame<msrh01::br> KinematicModel::s_bodyToJointBackRight;
TrafoBodyToJointFrame<msrh01::bl> KinematicModel::s_bodyToJointBackLeft;

const TrafoBase* KinematicModel::legTrafos[] =
{
    &s_bodyToJointFrontRight,
    &s_bodyToJointFrontLeft,
    &s_bodyToJointMidRight,
    &s_bodyToJointMidLeft,
    &s_bodyToJointBackRight,
    &s_bodyToJointBackLeft
};

KinematicModel::KinematicModel()
{
    for (uint8_t i = 0; i < msrh01::legs; ++i)
    {
        legTrafos[i]->forward(Vector3d(msrh01::anglesInit[msrh01::coxaIndex],
                                       msrh01::anglesInit[msrh01::femurIndex],
                                       msrh01::anglesInit[msrh01::tibiaIndex]),
                              m_initFeet[i]);
        m_actualFoot[i] = m_initFeet[i];
    }
}

KinematicModel::~KinematicModel()
{
}

trafoStatus KinematicModel::calcJointAngles(Vector3d foot, const Pose3d& body, const uint8_t legIndex, Vector3d& footJoint)
{
    m_actualFoot[legIndex] = foot;

    // Add desired body translation
    foot -= body.m_position;

    // Rotate to desired body orientation
    foot = math::rotateToOrientation(foot, body.m_orientation);

    // Get new angle vector of foot in joint frame
    trafoStatus result = legTrafos[legIndex]->backward(foot, footJoint);

    if (result != trafoOk)
    {
        Serial.print("Transformation error: ");
        Serial.println(result);
    }

    return result;
}
