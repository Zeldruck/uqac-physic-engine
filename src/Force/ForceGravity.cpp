#include "Force/ForceGravity.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

void ForceGravity::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	particle->AddForce(m_gravity * particle->mass);
}

void ForceGravity::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	rigidbody->AddForce(m_gravity * rigidbody->mass);
}