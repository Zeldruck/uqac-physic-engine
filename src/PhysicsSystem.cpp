#include <iostream>

#include "PhysicsSystem.hpp"
#include "Particle.hpp"
#include "EulerIntegrator.hpp"

PhysicsSystem::PhysicsSystem()
{
	m_integrator = new EulerIntegrator();
}

void PhysicsSystem::Update(float deltaTime, bool isGravityEnabled)
{
	m_integrator->Update(m_particles, deltaTime, isGravityEnabled);	
}

void PhysicsSystem::AddParticle(std::shared_ptr<Particle> particle)
{
	m_particles.push_back(particle);
}

void PhysicsSystem::RemoveParticle(std::shared_ptr<Particle> particle)
{
	for (auto it = m_particles.begin(); it != m_particles.end(); ++it)
	{
		if (*it == particle)
		{
			m_particles.erase(it);
			return;
		}
	}
}

void PhysicsSystem::PrintParticles()
{
	for (const std::shared_ptr<Particle> particle : m_particles)
	{
		std::cout << "Particle position: " << particle->position << std::endl;
		std::cout << "Particle velocity: " << particle->velocity << std::endl;
		std::cout << "Particle acceleration: " << particle->acceleration << std::endl;
	}
}