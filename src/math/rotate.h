#ifndef ROTATE_H
#define ROTATE_H

#include "vector3d.h"
#include "orientation3d.h"

namespace math
{

Vector3d rotateX(const Vector3d& input, float angle);

Vector3d rotateY(const Vector3d& input, float angle);

Vector3d rotateZ(const Vector3d& input, float angle);

Vector3d rotateToOrientation(const Vector3d& input, const Orientation3d& orientation);

}

#endif // ROTATE_H
