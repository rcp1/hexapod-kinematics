#include "trafoCoordBodyToHip.h"

TrafoCoordBodyToHip::TrafoCoordBodyToHip()
{
}

TrafoCoordBodyToHip::~TrafoCoordBodyToHip()
{
}

trafoStatus TrafoCoordBodyToHip::forward(const Vector3d& input, Vector3d& output, uint8_t legIndex) const
{
	trafoStatus result = trafoOk;
	const Vector3d toHip = this->createVector(legIndex);
	const Vector3d translatedInput = input - toHip;
	const TrafoRotation trafoRotation('z', getPhi0(legIndex));

	trafoRotation.forward(translatedInput, output);

	return result;
}

trafoStatus TrafoCoordBodyToHip::backward(const Vector3d& input, Vector3d& output, uint8_t legIndex) const
{
	trafoStatus result = trafoOk;
	const Vector3d toHip = this->createVector(legIndex);
	const TrafoRotation trafoRotation('z', getPhi0(legIndex));
	Vector3d rotatedInput;

	trafoRotation.backward(input, rotatedInput);

	output = rotatedInput + toHip;


	return result;
}

float TrafoCoordBodyToHip::getPhi0(uint8_t legIndex) const
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

Vector3d TrafoCoordBodyToHip::createVector(uint8_t legIndex) const
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
