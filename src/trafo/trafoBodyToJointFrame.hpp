#ifndef TRAFOBODYTOJOINT_H
#define TRAFOBODYTOJOINT_H

#include "trafoBase.h"
#include "trafoBodyToHipFrame.hpp"
#include "trafoHipToJointFrame.h"
#include <stdint.h>

template<uint8_t t_legIndex>
class TrafoBodyToJointFrame : public TrafoBase
{
public:
    TrafoBodyToJointFrame();

    virtual ~TrafoBodyToJointFrame();

    /// Forward transformation. Transforms input to output in a "higher" coordinate system.
    /// \param[in]  input       Reference to the input vector
    /// \param[out] output      Reference to the output vector
    /// \return shows success or error
    virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const;

    /// Backward transformation. Transforms input to output in a "lower" coordinate system.
    /// \param[in]  input       Pointer to the input vector
    /// \param[out] output      Pointer to the output vector
    /// \return shows success or error
    virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const;

protected:
    TrafoBodyToHipFrame<t_legIndex> m_bodyToHip;
    TrafoHipToJointFrame m_hipToJoint;
};

template<uint8_t t_legIndex>
TrafoBodyToJointFrame<t_legIndex>::TrafoBodyToJointFrame()
{
}

template<uint8_t t_legIndex>
TrafoBodyToJointFrame<t_legIndex>::~TrafoBodyToJointFrame()
{
}

template<uint8_t t_legIndex>
trafoStatus TrafoBodyToJointFrame<t_legIndex>::forward(const Vector3d& input, Vector3d& output) const
{
    trafoStatus result = trafoOk;

    result = m_hipToJoint.forward(input, output);
    if (result)
        return result;

    result = m_bodyToHip.backward(output, output);

    return result;
}

template<uint8_t t_legIndex>
trafoStatus TrafoBodyToJointFrame<t_legIndex>::backward(const Vector3d& input, Vector3d& output) const
{
    trafoStatus result = trafoOk;

    result = m_bodyToHip.forward(input, output);
    if (result)
        return result;

    result = m_hipToJoint.backward(output, output);

    return result;
}

#endif // TRAFOBODYTOJOINT_H
