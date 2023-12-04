#include "Collision/Primitives/Primitive.hpp"
#include "Rigidbody.hpp"

Primitive::Primitive(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset)
{
	this->rigidbody = rigidbody;
	this->offset = offset;
}
