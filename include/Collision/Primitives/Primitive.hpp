#pragma once
#include "Matrix4.hpp"

#include <memory>
#include <string>

class Rigidbody;

class Primitive
{
public:
	Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset);

public:
	std::shared_ptr<Rigidbody> rigidbody;
	Matrix4f offset;
};