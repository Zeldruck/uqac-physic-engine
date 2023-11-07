#include "Rigidbody.hpp"
#include "Constants/PhysicConstants.hpp"

Rigidbody::Rigidbody()
	:
	transform(Transform()),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	momentOfInertia(Vector3f::Zero),
	mass(MIN_MASS)
{
}

Rigidbody::Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name)
	:
	transform(transform),
	velocity(velocity),
	m_acceleration(acceleration),
	force(Vector3f::Zero),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	angularVelocity(angularVelocity),
	m_angularAcceleration(angularAcceleration),
	momentOfInertia(momentOfInertia),
	name(name)
{
}

void Rigidbody::ClearForce()
{
	force = Vector3f::Zero;
}

void Rigidbody::AddForce(const Vector3f& f)
{
	force += f;
}

void Rigidbody::RemoveForce(const Vector3f& f)
{
	force -= f;
}

void Rigidbody::AddTorque(const Vector3f& t)
{
	torque += t;
}

void Rigidbody::RemoveTorque(const Vector3f& t)
{
	torque -= t;
}

Vector3f const Rigidbody::GetAcceleration()
{
	m_acceleration = force / mass;
	return m_acceleration;
}

void Rigidbody::SetAcceleration(const Vector3f& acceleration)
{
	m_acceleration = acceleration;
}

Vector3f const Rigidbody::GetAngularAcceleration()
{
	if (momentOfInertia.x == 0.f && momentOfInertia.y == 0.f && momentOfInertia.z == 0.f)
		return Vector3f::Zero;

	m_angularAcceleration = torque / momentOfInertia;
	return m_angularAcceleration;
}

void Rigidbody::SetAngularAcceleration(const Vector3f& acceleration)
{
	m_angularAcceleration = acceleration;
}