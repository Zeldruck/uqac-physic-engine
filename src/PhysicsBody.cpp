#include <PhysicsBody.hpp>
#include "Constants/PhysicConstants.hpp"

PhysicsBody::PhysicsBody() :
	mass(MIN_MASS),
	velocity(Vector3f::Zero),
	acceleration(Vector3f::Zero),
	force(Vector3f::Zero)
{
}

PhysicsBody::PhysicsBody(Vector3f velocity, Vector3f acceleration, float mass) :
	mass(mass <= 0.0f ? MIN_MASS : mass),
	velocity(velocity),
	acceleration(acceleration),
	force(Vector3f::Zero)
{
}

void PhysicsBody::ClearForce()
{
	force = Vector3f::Zero;
}

void PhysicsBody::AddForce(const Vector3f& f)
{
	force += f;
}

void PhysicsBody::RemoveForce(const Vector3f& f)
{
	force -= f;
}

Vector3f const PhysicsBody::GetAcceleration()
{
	acceleration = force / mass;
	return acceleration;
}

Vector3f PhysicsBody::GetPosition() const
{
	return Vector3f::Zero;
}