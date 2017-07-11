#ifndef TRAFOBODYTOLEG_H
#define TRAFOBODYTOLEG_H

#include "pose3d.h"
#include "trafoBase.h"
#include "trafoKin3AxisLeg.h"
#include "trafoCoordBodyToHip.h"

class TrafoCoordBodyToLeg : public TrafoBase
{
public:
    TrafoCoordBodyToLeg();
    virtual ~TrafoCoordBodyToLeg();

    /// Forward transformation. Transforms input to output in a "higher" coordinate system.
    /// \param[in]  input       Reference to the input vector
    /// \param[out] output      Reference to the output vector
    /// \return shows success or error
    trafoStatus forward(const Pose3d& desMCSBody, const Vector3d& actBCSTCP, Pose3d& output, uint8_t legIndex) const;

    /// TODO
    virtual trafoStatus forward(const Vector3d& input, Vector3d& output) const;

    /// Backward transformation. Transforms input to output in a "lower" coordinate system.
    /// \param[in]  input       Reference to the input vector
    /// \param[out] output      Reference to the output vector
    /// \return shows success or error
    trafoStatus backward(const Pose3d& input, Pose3d& output, uint8_t legIndex) const;

    /// TODO
    virtual trafoStatus backward(const Vector3d& input, Vector3d& output) const;

protected:
    TrafoKin3AxisLeg m_trafoKin3AxisLeg;
    TrafoCoordBodyToHip m_trafoCoordBodyToHip;
};

#endif // TRAFOBODYTOLEG_H
