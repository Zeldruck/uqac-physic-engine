#include "Particle.hpp"
#include "Constants/PhysicConstants.hpp"

Particle::Particle()
    : PhysicsBody("Particle"),
    position(0.0f, 0.0f, 0.0f)
{
}

Particle::Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name)
    : PhysicsBody(velocity, acceleration, mass, name),
	position(position)
{
}