#pragma once
#include "Collision/Primitives/Primitive.hpp"
#include "Vector3.hpp"

class Box : public Primitive
{
public:
	Box(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const Vector3f halfSize);

public:
	Vector3f halfSize;
};