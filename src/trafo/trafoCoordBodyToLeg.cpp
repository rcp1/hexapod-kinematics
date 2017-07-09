#include "trafoCoordBodyToLeg.h"

TrafoCoordBodyToLeg::TrafoCoordBodyToLeg()
{
}


TrafoCoordBodyToLeg::~TrafoCoordBodyToLeg()
{
}

trafoStatus TrafoCoordBodyToLeg::forward(const Pose3d& desiredMCSBody, const Vector3d& actBCSTCP, Pose3d& output, uint8_t legIndex) const
{
	TrafoRotation trafoRotation;
	trafoStatus result = trafoOk;

	const Vector3d initBodyMCS(0.0f, 0.0f, 0.0f);
	Vector3d initBCS;
	Vector3d initMCS;

	// Get position vector of TCP in MCS
	result = m_trafoCoordBodyToHip.backward(actBCSTCP, initMCS, legIndex);
	if (result)
		return result;

	// Get difference vector of body in MCS
	const Vector3d diffBodyMCS = initBodyMCS - desiredMCSBody.m_position;

	// Get new position vector of TCP in MCS
	output.m_position = initMCS + diffBodyMCS;

	// Yaw
	trafoRotation.setAxisAndAngle('z', desiredMCSBody.m_orientation.psi);
	result = trafoRotation.forward(output.m_position, output.m_position);
	// Pitch
	trafoRotation.setAxisAndAngle('y', desiredMCSBody.m_orientation.theta);
	result = trafoRotation.forward(output.m_position, output.m_position);
	// Roll
	trafoRotation.setAxisAndAngle('x', desiredMCSBody.m_orientation.phi);
	result = trafoRotation.forward(output.m_position, output.m_position);

	// Get new position vector of TCP in BCS
	result = m_trafoCoordBodyToHip.forward(output.m_position, output.m_position, legIndex);
	if (result)
		return result;

	output.m_orientation = desiredMCSBody.m_orientation;

	return result;
}

trafoStatus TrafoCoordBodyToLeg::backward(const Pose3d& input, Pose3d& output, uint8_t legIndex) const
{
	TrafoRotation trafoCoordRotation;
	trafoStatus result = trafoOk;

	const Vector3d initACS(0.0f, 0.0f, 0.0f);
	const Vector3d initBodyMCS(0.0f, 0.0f, 0.0f);
	Vector3d initBCS;
	Vector3d bodyBCS;
	Vector3d newBCS;

	// Get new position vector of TCP in BCS
	result = m_trafoCoordBodyToHip.backward(input.m_position, newBCS, legIndex);
	if (result)
		return result;

	// Roll
	trafoCoordRotation.setAxisAndAngle('x', input.m_orientation.phi);
	result = trafoCoordRotation.backward(newBCS, newBCS);
	// Pitch
	trafoCoordRotation.setAxisAndAngle('y', input.m_orientation.theta);
	result = trafoCoordRotation.backward(newBCS, newBCS);
	// Yaw
	trafoCoordRotation.setAxisAndAngle('z', input.m_orientation.psi);
	result = trafoCoordRotation.backward(newBCS, newBCS);

	// Get new position vector of TCP in BCS
	result = m_trafoCoordBodyToHip.forward(newBCS, newBCS, legIndex);
	if (result)
		return result;

	// Get init position vector of TCP in BCS
	result = m_trafoKin3AxisLeg.forward(initACS, initBCS);
	if (result)
		return result;

	// Get difference vector of TCP in BCS
	const Vector3d diffBCS = initBCS - newBCS;

	// Get init position vector of body in BCS
	result = m_trafoCoordBodyToHip.forward(initBodyMCS, bodyBCS, legIndex);
	if (result)
		return result;

	// Get new position vector of body in BCS
	output.m_position = bodyBCS + diffBCS;

	// Get new position vector of body in MCS
	result = m_trafoCoordBodyToHip.backward(output.m_position, output.m_position, legIndex);

	output.m_orientation = input.m_orientation;
	return result;
}

trafoStatus TrafoCoordBodyToLeg::forward(const Vector3d& input, Vector3d& output) const
{
	trafoStatus result = trafoErrorNotImplemented;

	return result;
}

trafoStatus TrafoCoordBodyToLeg::backward(const Vector3d& input, Vector3d& output) const
{
	trafoStatus result = trafoErrorNotImplemented;

	return result;
}
