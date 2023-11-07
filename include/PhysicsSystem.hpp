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

class PhysicsSystem
{
public:
	PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry);
	PhysicsSystem(PhysicsSystem&&) = default;
	PhysicsSystem(const PhysicsSystem&) = delete;

	PhysicsSystem& operator=(PhysicsSystem&&) = delete;
	PhysicsSystem& operator=(const PhysicsSystem&) = delete;

	void Update(float deltaTime, bool isGravityEnabled);
	void AddParticle(std::shared_ptr<Particle> particle);
	void RemoveParticle(std::shared_ptr<Particle> particle);
	void PrintParticles();

	void AddRigidbody(std::shared_ptr<Rigidbody> rigidbody);
	void RemoveRigidbody(std::shared_ptr<Rigidbody> rigidbody);
	void PrintRigidbodies();

private:
	std::vector<std::shared_ptr<Particle>> m_particles;
	std::vector<std::shared_ptr<Rigidbody>> m_rigidbodies;
	std::shared_ptr<ForceRegistry> m_forceRegistry;
	//std::shared_ptr<ContactRegistry> m_contactRegistry;
	//std::vector<ParticleContactGenerator*> contactGenerators;
	//ParticleContactResolver contactResolver;
	std::unique_ptr<EulerIntegrator> m_integrator;
};