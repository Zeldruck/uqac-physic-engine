#pragma once
#include "Primitive.hpp"

class Sphere : public Primitive
{
public:
	Sphere(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const float radius);

public:
	float radius;
};

