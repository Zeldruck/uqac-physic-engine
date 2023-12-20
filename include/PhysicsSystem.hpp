#pragma once
#include <memory>
#include <vector>
#include <map>
#include "Force/ForceRegistry.hpp"
#include "Contact/ParticleContactGenerator.hpp"
#include "Contact/ParticleContactResolver.hpp"
#include "EulerIntegrator.hpp"

class Particle;
class Rigidbody;
class BVHNode;
struct PotentialContact;
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

	void Update(State& current, float deltaTime, bool isGravityEnabled, bool detectCollisions = true);
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

	void ClearForces();
	void BroadPhaseCollisionDetection();


private:
	std::vector<std::shared_ptr<Particle>> m_particles;
	std::vector<std::shared_ptr<Rigidbody>> m_rigidbodies;
	std::shared_ptr<ForceRegistry> m_forceRegistry;
	//std::shared_ptr<ContactRegistry> m_contactRegistry;
	//std::vector<ParticleContactGenerator*> contactGenerators;
	//ParticleContactResolver contactResolver;
	std::unique_ptr<EulerIntegrator> m_integrator;
	std::shared_ptr<BVHNode> m_rootBVHNode;
	PotentialContact* m_potentialContact;
	unsigned int m_potentialContactCount;
};