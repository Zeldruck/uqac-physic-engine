#pragma once
#include <memory>
#include <vector>

class Particle;
class EulerIntegrator;

class PhysicsSystem
{
public:
	PhysicsSystem();
	PhysicsSystem(PhysicsSystem&&) = default;
	PhysicsSystem(const PhysicsSystem&) = delete;

	PhysicsSystem& operator=(PhysicsSystem&&) = delete;
	PhysicsSystem& operator=(const PhysicsSystem&) = delete;

	void Update(float deltaTime, bool isGravityEnabled);
	void AddParticle(std::shared_ptr<Particle> particle);
	void RemoveParticle(std::shared_ptr<Particle> particle);
	void PrintParticles();

private:
	static PhysicsSystem* s_instance;
	std::vector<std::shared_ptr<Particle>> m_particles;
	EulerIntegrator* m_integrator;
};