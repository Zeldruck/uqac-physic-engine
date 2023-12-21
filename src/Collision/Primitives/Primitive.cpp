#include "Collision/Primitives/Primitive.hpp"
#include "Rigidbody.hpp"

Primitive::Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset)
	: type(PrimitiveType::TypePrimitive)
{
	this->rigidbody = rigidbody;
	this->offset = offset;
}

Primitive::Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const PrimitiveType& type)
	: type(type)
{
	this->rigidbody = rigidbody;
	this->offset = offset;
}

PrimitiveType Primitive::GetType() const
{
	return type;
}