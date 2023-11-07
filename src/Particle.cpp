#include "Particle.hpp"
#include "Constants/PhysicConstants.hpp"

Particle::Particle() 
	: 
	position(Vector3f::Zero),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	mass(MIN_MASS),
	name(std::string("Particle"))
{
}

Particle::Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name)
    :
	position(position),
	velocity(velocity),
	m_acceleration(acceleration),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	name(name)
{
}

void Particle::ClearForce()
{
	force = Vector3f::Zero;
}

void Particle::AddForce(const Vector3f& f)
{
	force += f;
}

void Particle::RemoveForce(const Vector3f& f)
{
	force -= f;
}

Vector3f const Particle::GetAcceleration()
{
	m_acceleration = force / mass;
	return m_acceleration;
}

Vector3f Particle::SetAcceleration(const Vector3f& acceleration)
{
	m_acceleration = acceleration;
	return m_acceleration;
}