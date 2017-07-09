#ifndef ANGLES_H
#define ANGLES_H

namespace math
{

const float degToRadFactor = 0.01745329251994329f; ///> pi / 180
const float radToDegFactor = 1.f / degToRadFactor; ///> 180 / pi

inline
float deg2rad(const float degVal)
{
	return (degToRadFactor * degVal);
}

inline
float rad2deg(const float radVal)
{
	return (radToDegFactor * radVal);
}

}

#endif // ANGLES_H
