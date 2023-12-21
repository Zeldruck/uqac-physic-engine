#pragma once
#include "Matrix4.hpp"

#include <memory>

class Rigidbody;

enum PrimitiveType
{
	TypeSphere,
	TypePlane,
	TypeBox,
	TypePrimitive
};

class Primitive
{
public:
	Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset);
	Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const PrimitiveType& type);
	virtual PrimitiveType GetType() const;

public:
	std::shared_ptr<Rigidbody> rigidbody;
	Matrix4f offset;
	PrimitiveType type;
};