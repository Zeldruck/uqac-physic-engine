#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include <iostream>

void EulerIntegrator::Integrate(float deltaTime)
{
	for (Particle& particle : particles)
	{
		particle.position += particle.velocity * deltaTime;
		particle.velocity += particle.acceleration * deltaTime;
	}
}

void EulerIntegrator::AddParticle(Particle& particle)
{
	particles.push_back(particle);
}

void EulerIntegrator::RemoveParticle(Particle& particle)
{
	for(auto& it = particles.begin(); it != particles.end(); ++it)
	{
		if(&(*it) == &particle)
		{
			particles.erase(it);
			return;
		}
	}
}

void EulerIntegrator::PrintParticles()
{
	for(const Particle& particle : particles)
	{
		std::cout << "Particle position: " << particle.position << std::endl;
		std::cout << "Particle velocity: " << particle.velocity << std::endl;
		std::cout << "Particle acceleration: " << particle.acceleration << std::endl;
	}
}