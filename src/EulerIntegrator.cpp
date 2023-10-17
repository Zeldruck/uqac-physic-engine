#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include <vector>

void EulerIntegrator::Update(std::vector<std::shared_ptr<Particle>>& particles, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	for (std::shared_ptr<Particle> particle : particles)
	{
		particle->velocity += particle->acceleration * deltaTime;
		particle->position += particle->velocity * deltaTime;
		//if(isGravityEnabled)
		//	particle->velocity += (particle->acceleration + g) * deltaTime;
		//else
	}
}