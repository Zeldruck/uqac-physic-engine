#include "Particle.hpp"

Particle::Particle()
    : name("Particle"), 
    position(0.0f, 0.0f, 0.0f), 
    velocity(0.0f, 0.0f, 0.0f), 
    acceleration(0.0f, 0.0f, 0.0f), 
    mass(MIN_MASS)
{
}

Particle::Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float m, std::string name)
	: name(name),
	position(position),
	velocity(velocity),
	acceleration(acceleration)
{
    mass = m <= 0.0f ? MIN_MASS : m;
}

void Particle::AddForce(const Vector3f& force)
{
    // Principe d'Alembert
    acceleration += force / mass;
}

void Particle::RemoveForce(const Vector3f& force)
{
    acceleration -= force / mass;
}

void Particle::ClearForce()
{
    acceleration = Vector3f(0.0f, 0.0f, 0.0f);
}