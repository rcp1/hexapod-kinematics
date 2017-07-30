#ifndef TRAFOHIPTOJOINT_H
#define TRAFOHIPTOJOINT_H

#include "trafoBase.h"

class TrafoHipToJointFrame : public TrafoBase
{
public:
    TrafoHipToJointFrame();

    ~TrafoHipToJointFrame();

    /// Forward transformation. Transforms input to output in a "higher" coordinate system.
    /// \param[in]  input       Reference to the input vector
    /// \param[out] output      Reference to the output vector
    /// \return shows success or error
    virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const;

    /// Backward transformation. Transforms input to output in a "lower" coordinate system.
    /// \param[in]  input       Reference to the input vector
    /// \param[out] output      Reference to the output vector
    /// \return shows success or error
    virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const;

protected:
    trafoStatus checkAngles(const float& phi1, const float& phi2, const float& phi3) const;

    static const float m_femurLength;
    static const float m_tibiaLength;
    static const float m_coxaLength;
};

#endif // TRAFOHIPTOJOINT_H
