#include <iostream>

#include <memory>
#include "PhysicsSystem.hpp"
#include "PhysicsBody.hpp"

PhysicsSystem::PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry) :
	m_forceRegistry(forceRegistry),
	m_integrator(std::make_unique<EulerIntegrator>())
{
}

void PhysicsSystem::Update(float deltaTime, bool isGravityEnabled)
{
	// Clear force in all particles
	for (std::shared_ptr<PhysicsBody> physicsBody : m_physicsbodies)
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
	m_integrator->Update(m_physicsbodies, deltaTime, isGravityEnabled);	

}

void PhysicsSystem::AddParticle(std::shared_ptr<PhysicsBody> particle)
{
	m_physicsbodies.push_back(particle);
	std::cout << particle->GetPosition() << std::endl;
}

void PhysicsSystem::RemoveParticle(std::shared_ptr<PhysicsBody> particle)
{
	for (auto it = m_physicsbodies.begin(); it != m_physicsbodies.end(); ++it)
	{
		if (*it == particle)
		{
			m_physicsbodies.erase(it);
			return;
		}
	}
}

void PhysicsSystem::PrintParticles()
{
	for (const std::shared_ptr<PhysicsBody> particle : m_physicsbodies)
	{
		std::cout << "Particle position: " << particle->GetPosition() << std::endl;
		std::cout << "Particle velocity: " << particle->velocity << std::endl;
		std::cout << "Particle acceleration: " << particle->GetAcceleration() << std::endl;
	}
}