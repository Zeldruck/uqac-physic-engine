#pragma once

#include "Vector3.hpp"
#include "Quaternion.hpp"

class Transform
{
public:
	Transform();
	Transform::Transform(Vector3f position, Quaternionf rotation, Vector3f scale);

	Vector3f position;
	Quaternionf rotation;
	Vector3f scale;
};