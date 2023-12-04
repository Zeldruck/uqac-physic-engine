#include "Collision/Primitives/Box.hpp"

Box::Box(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const Vector3f halfSize) :
	Primitive(rigidbody, offset)
{
	this->halfSize = halfSize;
}
