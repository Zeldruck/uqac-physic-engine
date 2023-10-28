#include "Rigidbody.hpp"

Rigidbody::Rigidbody()
	: PhysicsBody("Rigidbody"),
	transform(Transform()),
	angularVelocity(0.0f, 0.0f, 0.0f),
	angularAcceleration(0.0f, 0.0f, 0.0f),
	momentOfInertia(0.0f, 0.0f, 0.0f)
{
}

Rigidbody::Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name)
	: PhysicsBody(velocity, acceleration, mass, name),
	transform(transform),
	angularVelocity(angularVelocity),
	angularAcceleration(angularAcceleration),
	momentOfInertia(momentOfInertia)
{
}

Vector3f Rigidbody::GetPosition() const
{
	return transform.position;
}