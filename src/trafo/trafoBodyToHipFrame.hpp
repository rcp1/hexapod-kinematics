#ifndef TRAFOBODYTOHIP_H
#define TRAFOBODYTOHIP_H

#include "trafoBase.h"
#include "defines.h"
#include "rotate.h"

template<uint8_t t_legIndex>
class TrafoBodyToHipFrame : public TrafoBase
{
public:
    TrafoBodyToHipFrame();

    virtual ~TrafoBodyToHipFrame();

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
    const Vector3d m_translation;
    const float m_rotationAngle;

    static Vector3d getTranslation();
    static float getRotation();
};

template<uint8_t t_legIndex>
TrafoBodyToHipFrame<t_legIndex>::TrafoBodyToHipFrame() :
    m_translation(getTranslation()),
    m_rotationAngle(getRotation())
{
}

template<uint8_t t_legIndex>
TrafoBodyToHipFrame<t_legIndex>::~TrafoBodyToHipFrame()
{
}

template<uint8_t t_legIndex>
trafoStatus TrafoBodyToHipFrame<t_legIndex>::forward(const Vector3d& input, Vector3d& output) const
{
    const Vector3d translatedInput = input - m_translation;

    output = math::rotateZ(translatedInput, m_rotationAngle);

    return trafoOk;
}

template<uint8_t t_legIndex>
trafoStatus TrafoBodyToHipFrame<t_legIndex>::backward(const Vector3d& input, Vector3d& output) const
{
    Vector3d rotatedInput = math::rotateZ(input, -m_rotationAngle);

    output = rotatedInput + m_translation;

    return trafoOk;
}

template<uint8_t t_legIndex>
Vector3d TrafoBodyToHipFrame<t_legIndex>::getTranslation()
{
    Vector3d toHip;

    switch (t_legIndex)
    {
    default:
    case msrh01::legIndex::fr:
        toHip = Vector3d(msrh01::dimensions::bodyToCornerLegsX, msrh01::dimensions::bodyToCornerLegsY);
        break;
    case msrh01::legIndex::fl:
        toHip = Vector3d(msrh01::dimensions::bodyToCornerLegsX, -msrh01::dimensions::bodyToCornerLegsY);
        break;
    case msrh01::legIndex::mr:
        toHip = Vector3d(msrh01::dimensions::bodyToMidLegsX, msrh01::dimensions::bodyToMidLegsY);
        break;
    case msrh01::legIndex::ml:
        toHip = Vector3d(msrh01::dimensions::bodyToMidLegsX, -msrh01::dimensions::bodyToMidLegsY);
        break;
    case msrh01::legIndex::br:
        toHip = Vector3d(-msrh01::dimensions::bodyToCornerLegsX, msrh01::dimensions::bodyToCornerLegsY);
        break;
    case msrh01::legIndex::bl:
        toHip = Vector3d(-msrh01::dimensions::bodyToCornerLegsX, -msrh01::dimensions::bodyToCornerLegsY);
        break;
    }

    return toHip;
}

template<uint8_t t_legIndex>
float TrafoBodyToHipFrame<t_legIndex>::getRotation()
{
    switch (t_legIndex)
    {
    default:
    case msrh01::legIndex::fr:
        return msrh01::angles::fr;
        break;
    case msrh01::legIndex::fl:
        return msrh01::angles::fl;
        break;
    case msrh01::legIndex::mr:
        return msrh01::angles::mr;
        break;
    case msrh01::legIndex::ml:
        return msrh01::angles::ml;
        break;
    case msrh01::legIndex::br:
        return msrh01::angles::br;
        break;
    case msrh01::legIndex::bl:
        return msrh01::angles::bl;
        break;
    }
}

#endif // TRAFOBODYTOHIP_H
