#include "Collision/Primitives/Sphere.hpp"

Sphere::Sphere(const std::shared_ptr<Rigidbody>& rigidbody, const Matrix4f& offset, const float radius) :
	Primitive(rigidbody, offset)
{
	this->radius = radius;
}
