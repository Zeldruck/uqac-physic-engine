#include "Force/ForceGravity.hpp"
#include "Particle.hpp"

void ForceGravity::UpdateForce(Particle* particle, float deltaTime)
{
	particle->AddForce(m_gravity * particle->mass);
}