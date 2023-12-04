#pragma once
#include "Collision/Primitives/Primitive.hpp"
#include "Vector3.hpp"

#include <memory>

class Rigidbody;

class Plane : public Primitive
{
public:
	Plane(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const Vector3f normal, const float fOffset);

public:
	Vector3f normal;
	float fOffset;
};