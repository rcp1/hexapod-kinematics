#include "trafoCoordLegToLeg.h"

TrafoCoordLegToLeg::TrafoCoordLegToLeg() : m_trafoCoordReflectionX('x')
{
}


TrafoCoordLegToLeg::~TrafoCoordLegToLeg()
{
}

trafoStatus TrafoCoordLegToLeg::forward(const Vector3d& input, Vector3d& output, uint8_t legIndexTo) const
{
    trafoStatus result = trafoOk;
    const Vector3d toHip = this->createVector(legIndexTo);

    switch ((legIndexTo + 1) % 2)
    {
    case 0: // left side is reflected at x-Axis
        m_trafoCoordReflectionX.forward(input, output);
        output += toHip;
        break;
    case 1:
        output = input + toHip;
        break;
    }

    // outer legs should start at their init position too
    if (legIndexTo != msrh01::legIndex::mr && legIndexTo != msrh01::legIndex::ml)
    {
        float phi0 = getPhi0(legIndexTo);
        output.x += (msrh01::dimensions::coxa + msrh01::dimensions::femur) * sinf(phi0);
    }

    return result;
}

trafoStatus TrafoCoordLegToLeg::backward(const Vector3d& input, Vector3d& output, uint8_t legIndexTo) const
{
    return forward(input, output, legIndexTo);
}

float TrafoCoordLegToLeg::getPhi0(uint8_t legIndex) const
{
    switch (legIndex)
    {
    default:
    case msrh01::legIndex::fr:
        return msrh01::ANGLES::fr;
        break;
    case msrh01::legIndex::fl:
        return msrh01::ANGLES::fl;
        break;
    case msrh01::legIndex::mr:
        return msrh01::ANGLES::mr;
        break;
    case msrh01::legIndex::ml:
        return msrh01::ANGLES::ml;
        break;
    case msrh01::legIndex::br:
        return msrh01::ANGLES::br;
        break;
    case msrh01::legIndex::bl:
        return msrh01::ANGLES::bl;
        break;
    }
}

Vector3d TrafoCoordLegToLeg::createVector(uint8_t legIndex) const
{
    Vector3d toHip;

    switch (legIndex)
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
