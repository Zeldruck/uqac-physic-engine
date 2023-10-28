#pragma once
#include <memory>
#include <vector>
#include <map>
#include "Force/ForceRegistry.hpp"
#include "Contact/ParticleContactGenerator.hpp"
#include "Contact/ParticleContactResolver.hpp"

class PhysicsBody;
class EulerIntegrator;

class PhysicsSystem
{
public:
	//PhysicsSystem();
	PhysicsSystem(std::shared_ptr<ForceRegistry> forceRegistry);
	PhysicsSystem(PhysicsSystem&&) = default;
	PhysicsSystem(const PhysicsSystem&) = delete;

	PhysicsSystem& operator=(PhysicsSystem&&) = delete;
	PhysicsSystem& operator=(const PhysicsSystem&) = delete;

	void Update(float deltaTime, bool isGravityEnabled);
	void AddParticle(std::shared_ptr<PhysicsBody> particle);
	void RemoveParticle(std::shared_ptr<PhysicsBody> particle);
	void PrintParticles();

private:
	//static PhysicsSystem* s_instance;
	std::vector<std::shared_ptr<PhysicsBody>> m_particles;
	std::shared_ptr<ForceRegistry> m_forceRegistry;
	//std::shared_ptr<ContactRegistry> m_contactRegistry;
	//std::vector<ParticleContactGenerator*> contactGenerators;
	//ParticleContactResolver contactResolver;
	EulerIntegrator* m_integrator;
};