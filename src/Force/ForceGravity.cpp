#include "Force/ForceGravity.hpp"
#include "Particle.hpp"

void ForceGravity::UpdateForce(std::shared_ptr<PhysicsBody> particle, float deltaTime)
{
	particle->AddForce(m_gravity * particle->mass);
}