#pragma once
#include <memory>
#include <vector>
#include <map>
#include "Force/ForceRegistry.hpp"
#include "Contact/ParticleContactGenerator.hpp"
#include "Contact/ParticleContactResolver.hpp"
#include "Collision/ContactGenerator.hpp"
#include "Collision/ContactResolver.hpp"
#include "EulerIntegrator.hpp"

class Particle;
class Rigidbody;
class BVHNode;
struct PotentialContact;
struct PotentialContactPrimitive;
struct State;

class PhysicsSystem
{
public:
	PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry);
	PhysicsSystem(PhysicsSystem&&) = default;
	PhysicsSystem(const PhysicsSystem&) = delete;
	~PhysicsSystem();

	PhysicsSystem& operator=(PhysicsSystem&&) = delete;
	PhysicsSystem& operator=(const PhysicsSystem&) = delete;

	void Update(State& current, float deltaTime, bool isGravityEnabled, bool hasToDetectBroadPhase = false, bool hasToDetectNarrowPhase = false, bool hasToResolveContact = false);
	void AddParticle(std::shared_ptr<Particle> particle);
	void RemoveParticle(std::shared_ptr<Particle> particle);
	std::vector<std::shared_ptr<Particle>> GetParticles();
	void PrintParticles();

	void AddRigidbody(std::shared_ptr<Rigidbody> rigidbody);
	void RemoveRigidbody(std::shared_ptr<Rigidbody> rigidbody);
	std::vector<std::shared_ptr<Rigidbody>> GetRigidbodies();
	void PrintRigidbodies();

	void AddRootBVHNode(std::shared_ptr<BVHNode> node);
	PotentialContact* GetPotentialContactArray() const;
	unsigned int GetPotentialContactCount() const;
	void ParsePotentialContacts();
	PotentialContactPrimitive* GetPotentialContactPrimitiveArray() const;
	unsigned int GetPotentialContactPrimitiveCount() const;
	void ParsePotentialContactsPrimitive();

	void ClearForces();

	void BroadPhaseCollisionDetection();
	void NarrowPhaseCollisionDetection();

private:
	std::vector<std::shared_ptr<Particle>> m_particles;
	std::vector<std::shared_ptr<Rigidbody>> m_rigidbodies;
	std::shared_ptr<ForceRegistry> m_forceRegistry;
	std::unique_ptr<EulerIntegrator> m_integrator;

	// Broad Phase Variables
	std::shared_ptr<BVHNode> m_rootBVHNode;
	PotentialContact* m_potentialContact;
	unsigned int m_potentialContactCount;
	PotentialContactPrimitive* m_potentialContactPrimitive;
	unsigned int m_potentialContactPrimitiveCount;

	// Narrow Phase Variables
	std::unique_ptr<ContactGenerator> m_contactGenerator;
	std::unique_ptr<ContactResolver> m_contactResolver;
};