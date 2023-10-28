#include "Particle.hpp"
#include "Constants/PhysicConstants.hpp"

Particle::Particle()
    : PhysicsBody(),
    name("Particle"), 
    position(0.0f, 0.0f, 0.0f)
{
}

Particle::Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name)
    : PhysicsBody(velocity, acceleration, mass),
    name(name),
	position(position)
{
}