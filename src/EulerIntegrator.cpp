#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include <iostream>

void EulerIntegrator::Integrate(const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	for (std::shared_ptr<Particle> particle : particles)
	{
		particle->position += particle->velocity * deltaTime;
		if(isGravityEnabled)
			particle->velocity += (particle->acceleration + g) * deltaTime;
		else
			particle->velocity += particle->acceleration * deltaTime;
	}
}

void EulerIntegrator::AddParticle(std::shared_ptr<Particle> particle)
{
	particles.push_back(particle);
}

void EulerIntegrator::RemoveParticle(std::shared_ptr<Particle> particle)
{
	for(auto it = particles.begin(); it != particles.end(); ++it)
	{
		if(*it == particle)
		{
			particles.erase(it);
			return;
		}
	}
}

void EulerIntegrator::PrintParticles()
{
	for(const std::shared_ptr<Particle> particle : particles)
	{
		std::cout << "Particle position: " << particle->position << std::endl;
		std::cout << "Particle velocity: " << particle->velocity << std::endl;
		std::cout << "Particle acceleration: " << particle->acceleration << std::endl;
	}
}