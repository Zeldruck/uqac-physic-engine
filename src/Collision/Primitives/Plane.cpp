#include "Collision/Primitives/Plane.hpp"
#include "Rigidbody.hpp"

Plane::Plane(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const Vector3f normal, const float fOffset) :
	Primitive(rigidbody, offset, PrimitiveType::TypePlane)
{
	this->normal = normal;
}
