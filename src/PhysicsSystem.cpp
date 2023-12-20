#include <iostream>
#include <map>
#include <vector>
#include <memory>
#include "PhysicsSystem.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"
#include "Collision/BVHNode.hpp"
#include "Collision/BoundingSphere.hpp"
#include "Collision/BoundingBox.hpp"
#include "Collision/BVHNode.hpp"

PhysicsSystem::PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry) :
	m_forceRegistry(forceRegistry),
	m_integrator(std::make_unique<EulerIntegrator>())
{
	m_potentialContact = new PotentialContact();
}

PhysicsSystem::~PhysicsSystem()
{
	delete(m_potentialContact);
}

void PhysicsSystem::Update(float deltaTime, bool isGravityEnabled, bool detectCollisions /* = true */)
{
	// Clear les forces des particles et rigidbodies
	ClearForces();

	// Mise à jour des forces
	m_forceRegistry->UpdateForces(deltaTime);

	// Mise à jour des particules
	m_integrator->Update(m_particles, m_rigidbodies, deltaTime, isGravityEnabled);	

	// Résolution des collisions
	if(detectCollisions)
		BroadPhaseCollisionDetection();
}

void PhysicsSystem::ClearForces()
{
	for (auto& particle : m_particles)
	{
		particle->ClearForce();
	}
	for (auto& rigidbody : m_rigidbodies)
	{
		rigidbody->ClearForce();
		rigidbody->ClearTorque();
	}
}

void PhysicsSystem::AddRootBVHNode(std::shared_ptr<BVHNode> node)
{
	m_rootBVHNode = node;
}

void PhysicsSystem::BroadPhaseCollisionDetection()
{
	m_rootBVHNode->RecalculateBoundingVolume();
	m_potentialContactCount = m_rootBVHNode->GetPotentialContact(m_potentialContact, 1000);
}

void PhysicsSystem::AddParticle(std::shared_ptr<Particle> particle)
{
	m_particles.push_back(particle);
	std::cout << particle->position << std::endl;
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

void PhysicsSystem::AddRigidbody(std::shared_ptr<Rigidbody> rigidbody)
{
	m_rigidbodies.push_back(rigidbody);
}

void PhysicsSystem::RemoveRigidbody(std::shared_ptr<Rigidbody> rigidbody)
{
	for (auto it = m_rigidbodies.begin(); it != m_rigidbodies.end(); ++it)
	{
		if (*it == rigidbody)
		{
			m_rigidbodies.erase(it);
			return;
		}
	}
}

std::vector<std::shared_ptr<Particle>> PhysicsSystem::GetParticles()
{
	return m_particles;
}

void PhysicsSystem::PrintParticles()
{
	for (const std::shared_ptr<Particle> particle : m_particles)
	{
		std::cout << "Particle position: " << particle->position << std::endl;
		std::cout << "Particle velocity: " << particle->velocity << std::endl;
		std::cout << "Particle acceleration: " << particle->GetAcceleration() << std::endl;
	}
}

std::vector<std::shared_ptr<Rigidbody>> PhysicsSystem::GetRigidbodies()
{
	return m_rigidbodies;
}

PotentialContact* PhysicsSystem::GetPotentialContactArray() const
{
	return m_potentialContact;
}

unsigned int PhysicsSystem::GetPotentialContactCount() const
{
	return m_potentialContactCount;
}

void PhysicsSystem::PrintRigidbodies()
{
	for (const std::shared_ptr<Rigidbody> rigidbody : m_rigidbodies)
	{
		std::cout << rigidbody->name << " position" << rigidbody->position << std::endl;
		std::cout << rigidbody->name << " rotation" << rigidbody->rotation << std::endl;
		std::cout << rigidbody->name << " scale" << rigidbody->scale << std::endl;
		std::cout << rigidbody->name << " velocity" << rigidbody->velocity << std::endl;
		std::cout << rigidbody->name << " acceleration" << rigidbody->GetAcceleration() << std::endl;
		std::cout << rigidbody->name << " angular velocity" << rigidbody->angularVelocity << std::endl;
		std::cout << rigidbody->name << " angular acceleration" << rigidbody->GetAngularAcceleration() << std::endl;
		std::cout << rigidbody->name << " torque" << rigidbody->torque << std::endl;
		std::cout << rigidbody->name << " mass"	 << rigidbody->mass << std::endl;
		std::cout << rigidbody->name << " force" << rigidbody->force << std::endl;
	}
}