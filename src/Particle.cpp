#include "Particle.hpp"
#include "Constants/PhysicConstants.hpp"

Particle::Particle() 
	: 
	name(std::string("Particle")),
	position(Vector3f::Zero),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	mass(MIN_MASS),
	force(Vector3f::Zero)
{
}

Particle::Particle(std::string name) :
	name(name),
	position(Vector3f::Zero),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	mass(MIN_MASS),
	force(Vector3f::Zero)
{
}

Particle::Particle(std::string name, Vector3f position) :
	name(name),
	position(position),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	mass(MIN_MASS),
	force(Vector3f::Zero)
{
}

Particle::Particle(std::string name, Vector3f position, float mass) :
	name(name),
	position(position),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	force(Vector3f::Zero)
{
}

Particle::Particle(std::string name, Vector3f position, float mass, Vector3f velocity) :
	name(name),
	position(position),
	velocity(velocity),
	m_acceleration(Vector3f::Zero),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	force(Vector3f::Zero)
{
}

Particle::Particle(std::string name, Vector3f position, float mass, Vector3f velocity, Vector3f acceleration) :
	name(name),
	position(position),
	velocity(velocity),
	m_acceleration(acceleration),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	force(Vector3f::Zero)
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