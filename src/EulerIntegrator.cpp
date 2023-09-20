#include "EulerIntegrator.hpp"
#include "Particle.hpp"

void EulerIntegrator::Integrate(Particle& particle, float deltaTime)
{
	particle.position += particle.velocity * deltaTime;
	particle.velocity += particle.acceleration * deltaTime;
}