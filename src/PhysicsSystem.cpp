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

#include "Collision/ContactGenerator.hpp"
#include "Collision/ContactResolver.hpp"

#include "Collision/Primitives/Primitive.hpp"
#include "Collision/Primitives/Sphere.hpp"
#include "Collision/Primitives/Box.hpp"
#include "Collision/Primitives/Plane.hpp"

#include "State.hpp"

PhysicsSystem::PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry) :
	m_forceRegistry(forceRegistry),
	m_integrator(std::make_unique<EulerIntegrator>()),
	m_contactGenerator(std::make_unique<ContactGenerator>(50)),
	m_contactResolver(std::make_unique<ContactResolver>(50)),
	m_potentialContactCount(0),
	m_potentialContactPrimitiveCount(0)
{
	m_potentialContact = new PotentialContact();
	m_potentialContactPrimitive = new PotentialContactPrimitive();
}

PhysicsSystem::~PhysicsSystem()
{
	if(m_potentialContact)
		delete(m_potentialContact);
	if(m_potentialContactPrimitive)
		delete(m_potentialContactPrimitive);
}

void PhysicsSystem::Update(State& current, float deltaTime, bool isGravityEnabled, bool hasToDetectCollisions /* = true */)
{
	// Clear les forces des particles et rigidbodies
	ClearForces();

	// Mise à jour des forces
	m_forceRegistry->UpdateForces(deltaTime);

	// Mise à jour des particules
	m_integrator->Update(current, m_particles, m_rigidbodies, deltaTime, isGravityEnabled);	

	// Résolution des collisions
	if (hasToDetectCollisions)
	{
		BroadPhaseCollisionDetection();
		NarrowPhaseCollisionDetection();
		m_contactResolver->ResolveContacts(m_contactGenerator->GetContacts(), deltaTime);
	}

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
	m_potentialContactPrimitiveCount = m_rootBVHNode->GetPotentialContactPrimitive(m_potentialContactPrimitive, 1000);
	ParsePotentialContacts();
	ParsePotentialContactsPrimitive();
}

void PhysicsSystem::NarrowPhaseCollisionDetection()
{
	for (unsigned int i = 0; i < m_potentialContactPrimitiveCount; i++)
	{
		PrimitiveType type1 = m_potentialContactPrimitive[i].primitives[0]->GetType();
		PrimitiveType type2 = m_potentialContactPrimitive[i].primitives[1]->GetType();

		if (type1 == PrimitiveType::TypeSphere && type2 == PrimitiveType::TypeSphere)
		{
			std::shared_ptr<Sphere> sphere1 = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[0]);
			std::shared_ptr<Sphere> sphere2 = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[1]);
			
			Sphere sphereA = *sphere1;
			Sphere sphereB = *sphere2;

			m_contactGenerator->DetectSandS(sphereA, sphereB);

			std::cout << "Sphere - Sphere" << std::endl;
			std::cout << "Sphere 1: " << sphere1->rigidbody->name << std::endl;
			std::cout << "Sphere 2: " << sphere2->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypeSphere && type2 == PrimitiveType::TypeBox)
		{
			std::shared_ptr<Sphere> sphere = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[0]);
			std::shared_ptr<Box> box = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[1]);

			Sphere sphereA = *sphere;
			Box boxB = *box;

			m_contactGenerator->DetectSandB(sphereA, boxB);

			std::cout << "Sphere - Box" << std::endl;
			std::cout << "Sphere: " << sphere->rigidbody->name << std::endl;
			std::cout << "Box: " << box->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypeBox && type2 == PrimitiveType::TypeSphere)
		{
			std::shared_ptr<Sphere> sphere = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[1]);
			std::shared_ptr<Box> box = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[0]);
			
			Sphere sphereA = *sphere;
			Box boxB = *box;

			m_contactGenerator->DetectSandB(sphereA, boxB);
			
			std::cout << "Box - Sphere" << std::endl;
			std::cout << "Sphere: " << sphere->rigidbody->name << std::endl;
			std::cout << "Box: " << box->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypeSphere && type2 == PrimitiveType::TypePlane)
		{
			std::shared_ptr<Sphere> sphere = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[0]);
			std::shared_ptr<Plane> plane = std::dynamic_pointer_cast<Plane>(m_potentialContactPrimitive[i].primitives[1]);

			Sphere sphereA = *sphere;
			Plane planeB = *plane;

			m_contactGenerator->DetectSandP(sphereA, planeB);

			std::cout << "Sphere - Plane" << std::endl;
			std::cout << "Sphere: " << sphere->rigidbody->name << std::endl;
			std::cout << "Plane: " << plane->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypePlane && type2 == PrimitiveType::TypeSphere)
		{
			std::shared_ptr<Sphere> sphere = std::dynamic_pointer_cast<Sphere>(m_potentialContactPrimitive[i].primitives[1]);
			std::shared_ptr<Plane> plane = std::dynamic_pointer_cast<Plane>(m_potentialContactPrimitive[i].primitives[0]);
			
			Sphere sphereA = *sphere;
			Plane planeB = *plane;

			m_contactGenerator->DetectSandP(sphereA, planeB);
			
			std::cout << "Plane - Sphere" << std::endl;
			std::cout << "Sphere: " << sphere->rigidbody->name << std::endl;
			std::cout << "Plane: " << plane->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypeBox && type2 == PrimitiveType::TypeBox)
		{
			std::shared_ptr<Box> box1 = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[0]);
			std::shared_ptr<Box> box2 = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[1]);
			
			Box boxA = *box1;
			Box boxB = *box2;

			m_contactGenerator->DetectBandB(boxA, boxB);
			
			std::cout << "Box - Box" << std::endl;
			std::cout << "Box 1: " << box1->rigidbody->name << std::endl;
			std::cout << "Box 2: " << box2->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypeBox && type2 == PrimitiveType::TypePlane)
		{
			std::shared_ptr<Box> box = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[0]);
			std::shared_ptr<Plane> plane = std::dynamic_pointer_cast<Plane>(m_potentialContactPrimitive[i].primitives[1]);
			
			Box boxA = *box;
			Plane planeB = *plane;

			m_contactGenerator->DetectBandP(boxA, planeB);
			
			std::cout << "Box - Plane" << std::endl;
			std::cout << "Box: " << box->rigidbody->name << std::endl;
			std::cout << "Plane: " << plane->rigidbody->name << std::endl;
		}
		else if (type1 == PrimitiveType::TypePlane && type2 == PrimitiveType::TypeBox)
		{
			std::shared_ptr<Box> box = std::dynamic_pointer_cast<Box>(m_potentialContactPrimitive[i].primitives[1]);
			std::shared_ptr<Plane> plane = std::dynamic_pointer_cast<Plane>(m_potentialContactPrimitive[i].primitives[0]);

			Box boxA = *box;
			Plane planeB = *plane;

			m_contactGenerator->DetectBandP(boxA, planeB);

			std::cout << "Plane - Box" << std::endl;
			std::cout << "Box: " << box->rigidbody->name << std::endl;
			std::cout << "Plane: " << plane->rigidbody->name << std::endl;
		}
	}
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

PotentialContactPrimitive* PhysicsSystem::GetPotentialContactPrimitiveArray() const
{
	return m_potentialContactPrimitive;
}

unsigned int PhysicsSystem::GetPotentialContactPrimitiveCount() const
{
	return m_potentialContactPrimitiveCount;
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

void PhysicsSystem::ParsePotentialContacts()
{
	for (unsigned int i = 0; i < m_potentialContactCount; ++i)
	{
		std::cout << "Contact " << i << std::endl;
		std::cout << "Rigidbody 1: " << m_potentialContact[i].rigidbodies[0]->name << std::endl;
		std::cout << "Rigidbody 2: " << m_potentialContact[i].rigidbodies[1]->name << std::endl;
	}
}

void PhysicsSystem::ParsePotentialContactsPrimitive()
{
	for (unsigned int i = 0; i < m_potentialContactPrimitiveCount; ++i)
	{
		std::cout << "Contact " << i << std::endl;
		std::cout << "Rigidbody 1: " << m_potentialContactPrimitive[i].primitives[0]->rigidbody->name << std::endl;
		std::cout << "Rigidbody 2: " << m_potentialContactPrimitive[i].primitives[1]->rigidbody->name << std::endl;
	}

}