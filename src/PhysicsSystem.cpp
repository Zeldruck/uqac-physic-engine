#include <iostream>

#include "PhysicsSystem.hpp"
#include "PhysicsBody.hpp"
#include "EulerIntegrator.hpp"

PhysicsSystem::PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry) :
	m_forceRegistry(forceRegistry),
	m_integrator(new EulerIntegrator())
{
}

void PhysicsSystem::Update(float deltaTime, bool isGravityEnabled)
{
	// Clear force in all particles
	for (std::shared_ptr<PhysicsBody> physicsBody : m_particles)
	{
		physicsBody->ClearForce();
	}

	// Mise à jour des forces
	m_forceRegistry->UpdateForces(deltaTime);

	// Génération des contacts
	//std::vector<ParticleContact> contacts;
	//for (auto generator : contactGenerators) {
	//	generator->GenerateContacts(contacts);
	//}

	// Résolution des contacts
	//contactResolver.ResolveContacts(contacts, deltaTime);

	// Mise à jour des particules
	m_integrator->Update(m_particles, deltaTime, isGravityEnabled);	

}

void PhysicsSystem::AddParticle(std::shared_ptr<PhysicsBody> particle)
{
	m_particles.push_back(particle);
	std::cout << particle->GetPosition() << std::endl;
}

void PhysicsSystem::RemoveParticle(std::shared_ptr<PhysicsBody> particle)
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
	for (const std::shared_ptr<PhysicsBody> particle : m_particles)
	{
		std::cout << "Particle position: " << particle->GetPosition() << std::endl;
		std::cout << "Particle velocity: " << particle->velocity << std::endl;
		std::cout << "Particle acceleration: " << particle->GetAcceleration() << std::endl;
	}
}