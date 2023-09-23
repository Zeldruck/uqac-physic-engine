#include "Particle.hpp"

Particle::Particle()
    : name("Particle"), 
    position(0.0f, 0.0f, 0.0f), 
    velocity(0.0f, 0.0f, 0.0f), 
    acceleration(0.0f, 0.0f, 0.0f), 
    mass(MIN_MASS)
{
}

Particle::Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name)
	: name(name),
	position(position),
	velocity(velocity),
	acceleration(acceleration)
{
    if (mass <= 0.0f)
    {
        mass = MIN_MASS;
    }
}