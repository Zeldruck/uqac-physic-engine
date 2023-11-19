#include "Transform.hpp"

Transform::Transform()
	: position(0.0f, 0.0f, 0.0f),
	rotation(0.0f, 0.0f, 0.0f, 1.0f),
	scale(1.0f, 1.0f, 1.0f)
{
}

Transform::Transform(Vector3f position, Quaternionf rotation, Vector3f scale)
	: position(position),
	rotation(rotation),
	scale(scale)
{
}