#include "servoTypes.h"

LegServoVector::LegServoVector(const uint8_t legIndex, const uint16_t moveTime)	:
	m_legIndex(legIndex),
	m_moveTime(moveTime),
	m_ACSTCP(Vector3d(0.0f, 0.0f, 0.0f))
{
	init();
}

LegServoVector::LegServoVector(const Vector3d& input, const uint16_t moveTime)
	: m_moveTime(moveTime)
{
	setTrafoVector(input);
}

void LegServoVector::init()
{
	for (uint8_t i = 0; i < msrh01::servosPerLeg; ++i)
	{
		m_servoVectors[i].pos = servoNullOffset + msrh01::servoOffsets[((m_legIndex)  * msrh01::servosPerLeg) + i];
		m_servoVectors[i].latchedPos = servoNullOffset + msrh01::servoOffsets[((m_legIndex)  * msrh01::servosPerLeg) + i];
	}
}

void LegServoVector::setTrafoVector(const Vector3d& input)
{
    m_ACSTCP = input;
	Vector3d temp = input;
    temp[1] -= (HALF_PI / 2.0f);
    temp[2] -= (HALF_PI / 2.0f);
	switch ((m_legIndex + 1) % 2)
	{
	case 0:
		temp[1] *= -1; // Angles are inverted for left side femur / tibia
		temp[2] *= -1; // Angles are inverted for left side femur / tibia
		break;
	case 1:
		break;
	}
	m_servoVectors[msrh01::tibiaIndex].latchedPos = m_servoVectors[msrh01::tibiaIndex].pos;
	m_servoVectors[msrh01::tibiaIndex].pos = max(min(((uint16_t)roundf(temp[2] * radToServoFactor) + servoNullOffset + msrh01::servoOffsets[(m_legIndex)  * msrh01::servosPerLeg + msrh01::tibiaIndex]), servoMaximum), servoMinimum);
	m_servoVectors[msrh01::femurIndex].latchedPos = m_servoVectors[msrh01::femurIndex].pos;
	m_servoVectors[msrh01::femurIndex].pos = max(min(((uint16_t)roundf(temp[1] * radToServoFactor) + servoNullOffset + msrh01::servoOffsets[(m_legIndex)  * msrh01::servosPerLeg + msrh01::femurIndex]), servoMaximum), servoMinimum);
	m_servoVectors[msrh01::coxaIndex].latchedPos = m_servoVectors[msrh01::coxaIndex].pos;
	m_servoVectors[msrh01::coxaIndex].pos = max(min(((uint16_t)roundf(temp[0] * radToServoFactor) + servoNullOffset + msrh01::servoOffsets[(m_legIndex)  * msrh01::servosPerLeg + msrh01::coxaIndex]), servoMaximum), servoMinimum);
}

Vector3d LegServoVector::getTrafoVector()
{
	return m_ACSTCP;
}

void LegServoVector::setLegIndex(const uint8_t legIndex)
{
	m_legIndex = legIndex;
	this->init();
}

void LegServoVector::setMoveTime(const uint16_t moveTime)
{
	m_moveTime = moveTime;
}

void LegServoVector::setServos()
{
#ifdef SSC32
    setServosSSC32();
#endif
#ifdef SD21
    setServosSD21();
#endif
}

#ifdef SSC32
void LegServoVector::setServosSSC32()
{
    STRING cmd;
    switch (m_legIndex)
    {
    default:
	case msrh01::legIndex::fr:
		cmd.concat("#24 P");
		cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
		cmd.concat(" #25 P");
		cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
		cmd.concat(" #26 P");
		cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
		cmd.concat(" T");
		cmd.concat((String(m_moveTime)));
		cmd.concat(" \r");
		break;
    case msrh01::legIndex::fl:
        cmd.concat("#8 P");
        cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
        cmd.concat(" #9 P");
        cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
        cmd.concat(" #10 P");
        cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
        cmd.concat(" T");
        cmd.concat((String(m_moveTime)));
        cmd.concat(" \r");
        break;
	case msrh01::legIndex::mr:
		cmd.concat("#20 P");
		cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
		cmd.concat(" #21 P");
		cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
		cmd.concat(" #22 P");
		cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
		cmd.concat(" T");
		cmd.concat((String(m_moveTime)));
		cmd.concat(" \r");
		break;
    case msrh01::legIndex::ml:
        cmd.concat("#4 P");
        cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
        cmd.concat(" #5 P");
        cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
        cmd.concat(" #6 P");
        cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
        cmd.concat(" T");
        cmd.concat((String(m_moveTime)));
        cmd.concat(" \r");
        break;
    case msrh01::legIndex::br:
        cmd.concat("#16 P");
        cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
        cmd.concat(" #17 P");
        cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
        cmd.concat(" #18 P");
        cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
        cmd.concat(" T");
        cmd.concat((String(m_moveTime)));
        cmd.concat(" \r");
        break;
	case msrh01::legIndex::bl:
		cmd.concat("#0 P");
		cmd.concat((String(m_servoVectors[msrh01::coxaIndex].pos)));
		cmd.concat(" #1 P");
		cmd.concat((String(m_servoVectors[msrh01::femurIndex].pos)));
		cmd.concat(" #2 P");
		cmd.concat((String(m_servoVectors[msrh01::tibiaIndex].pos)));
		cmd.concat(" T");
		cmd.concat((String(m_moveTime)));
		cmd.concat(" \r");
		break;
    }
    Serial3.write(cmd.c_str());
}
#endif

#ifdef SD21
void LegServoVector::setServosSD21()
{
#ifdef ARDUINO_PLATFORM
	Wire.beginTransmission(sd21AddressI2C);
	Wire.write((m_legIndex - 1)  * sd21CountRegistersPerLeg);
#endif
	for (uint8_t i = 0; i < msrh01::servosPerLeg; ++i)
	{
		uint8_t speed = 0;

		if (m_moveTime != 0)
		{
			speed = max((uint8_t)roundf((float)fabs((float)m_servoVectors[i].pos - (float)m_servoVectors[i].latchedPos) * sd21RefreshRate / (float)m_moveTime), 1);
		}

#ifdef ARDUINO_PLATFORM
		// Speed
		Wire.write(speed);
		// Low Byte
		Wire.write(m_servoVectors[i].lowerPos);
		// High Byte
		Wire.write(m_servoVectors[i].upperPos);
#else
		printf("Pos n-1:                   %4d \n", m_servoVectors[i].latchedPos);
		printf("Pos n:                     %4d \n", m_servoVectors[i].pos);
		printf("moveTime:                  %4d \n", m_moveTime);
		printf("Register Speed: %4d Byte: %4d \n", (m_legIndex - 1) * sd21CountRegistersPerLeg + (msrh01::servosPerLeg * i), speed);
		printf("Register Lower: %4d Byte: %4d \n", (m_legIndex - 1) * sd21CountRegistersPerLeg + (msrh01::servosPerLeg * i) + 1, m_servoVectors[i].lowerPos);
		printf("Register Upper: %4d Byte: %4d \n", (m_legIndex - 1) * sd21CountRegistersPerLeg + (msrh01::servosPerLeg * i) + 2, m_servoVectors[i].upperPos);
#endif
	}
#ifdef ARDUINO_PLATFORM
	Wire.endTransmission();
#endif
}
#endif
