#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include <vector>

void EulerIntegrator::Update(std::vector<std::shared_ptr<PhysicsBody>>& particles, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	for (std::shared_ptr<PhysicsBody> particle : particles)
	{
		Vector3f newPosition = particle->GetPosition() + particle->velocity * deltaTime;
		particle->SetPosition(newPosition);
		particle->velocity += particle->GetAcceleration() * deltaTime;
	}
}