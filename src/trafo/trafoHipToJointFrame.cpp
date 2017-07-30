#include "trafoHipToJointFrame.h"
#include "defines.h"
#include "compute.h"
#include "mathConstants.h"

const float TrafoHipToJointFrame::m_femurLength = msrh01::dimensions::femur;
const float TrafoHipToJointFrame::m_tibiaLength = msrh01::dimensions::tibia;
const float TrafoHipToJointFrame::m_coxaLength = msrh01::dimensions::coxa;

TrafoHipToJointFrame::TrafoHipToJointFrame()
{
}


TrafoHipToJointFrame::~TrafoHipToJointFrame()
{
}

trafoStatus TrafoHipToJointFrame::forward(const Vector3d& input, Vector3d& output) const
{
    using namespace math;

    trafoStatus result = trafoOk;
    float d, temp, TCPyproj, phi1, phi2, phi2s, phi2u, phi3 = 0.f;

    phi1 = input[0];
    phi2s = input[1];
    phi3 = fabs(HALF_PI - input[2]);

    // get third length of triangle BCTCP (d)
    d = sqrtf(power(m_femurLength, 2) + power(m_tibiaLength, 2) - 2.f * m_femurLength * m_tibiaLength * cosf(phi3));
    if (fabs(d) < math::epsilonFloat)
    {
        return trafoErrorSingularPosition; // d can't be 0 (no triangle)
    }

    // get xz-angle at B (phi2)
    temp = (power(m_femurLength, 2) + power(d, 2) - power(m_tibiaLength, 2)) / (2.f * m_femurLength * d);
    if (fabs(temp) > 1.f)
    {
        return trafoErrorSingularPosition; // acos is not defined for |x| > 1, catch possible failure (no triangle)
    }
    phi2 = acosf(temp);
    phi2u = phi2s - phi2;

    result = checkAngles(phi1, phi2, phi3);
    if (result)
        return result;

    output.z = sinf(phi2u) * d;
    TCPyproj = cosf(phi2u) * d;

    output.x = sinf(phi1) * (TCPyproj + m_coxaLength);
    output.y = cosf(phi1) * (TCPyproj + m_coxaLength);

    return result;
}

trafoStatus TrafoHipToJointFrame::checkAngles(const float& phi1, const float& phi2, const float& phi3) const
{
    if (phi1 >= HALF_PI || phi1 <= -HALF_PI)
        return trafoErrorInputOutOfBonds;
    if (phi2 >= PI || phi2 <= 0.f)
        return trafoErrorInputOutOfBonds;
    if (phi3 >= PI || phi3 <= 0.f)
        return trafoErrorInputOutOfBonds;
    return trafoOk;
}

trafoStatus TrafoHipToJointFrame::backward(const Vector3d& input, Vector3d& output) const
{
    using namespace math;

    trafoStatus result = trafoOk;
    float d, TCPyproj, temp, phi1, phi2u, phi2, phi3 = 0.f;

    // get xy-angle at B (phi1)
    phi1 = atan2f(input.x, input.y);

    TCPyproj = sqrtf(power(input.x, 2) + power(input.y, 2)) - m_coxaLength;

    // get third length of triangle BCTCP (d)
    d = sqrtf(power(input.z, 2) + power(TCPyproj, 2));
    if (d > (m_femurLength + m_tibiaLength))
    {
        return trafoErrorSingularPosition; // distance to goal point can't be greater than joint lengths together
    }
    if (fabs(d) < math::epsilonFloat)
    {
        return trafoErrorSingularPosition; // break here, otherwise zero division later
    }

    // get angle at C (phi3)
    temp = (power(m_femurLength, 2) + power(m_tibiaLength, 2) - power(d, 2)) / (2.f * m_femurLength * m_tibiaLength);
    if (fabs(temp) > 1.f)
    {
        return trafoErrorSingularPosition; // acos is not defined for |x| > 1, catch possible failure (no triangle)
    }
    phi3 = acosf(temp);

    // get xz-angle at B (phi2)
    temp = (power(m_femurLength, 2) + power(d, 2) - power(m_tibiaLength, 2)) / (2.f * m_femurLength * d);
    if ((float)fabs(temp) > 1.f)
    {
        return trafoErrorSingularPosition; // acos is not defined for |x| > 1, catch possible failure (no triangle)
    }
    phi2 = acosf(temp);

    if (input.y < 0.0)
        phi2u = atan2f(input.z, -TCPyproj);
    else
        phi2u = atan2f(input.z, TCPyproj);

    result = checkAngles(phi1, phi2, phi3);
    if (result)
        return result;

    output[0] = phi1;
    output[1] = phi2u + phi2;

    output[2] = HALF_PI - phi3; // Servo zero position is at 90 degrees

    return result;
}
