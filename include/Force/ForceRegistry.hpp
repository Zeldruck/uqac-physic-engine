#pragma once
#include <vector>
#include <memory>

class ForceGenerator;
class Particle;
class Rigidbody;

class ForceRegistry
{
private:
	struct ForceEntry
	{
		std::shared_ptr<Particle> particle;
		std::shared_ptr<ForceGenerator> forceGenerator;
	};

	struct ForceEntryRigidbody
	{
		std::shared_ptr<Rigidbody> rigidbody;
		std::shared_ptr<ForceGenerator> forceGenerator;
	};

	using Registry = std::vector<ForceEntry>;
	Registry m_registry;
	using RegistryRigidbody = std::vector<ForceEntryRigidbody>;
	RegistryRigidbody m_registryRigidbody;

public:
	void Add(std::shared_ptr<Particle> particle, std::shared_ptr<ForceGenerator> fg);
	void Add(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<ForceGenerator> fg);
	void Remove(std::shared_ptr<Particle> physicBody, std::shared_ptr<ForceGenerator> fg);
	void Remove(std::shared_ptr<Rigidbody> physicBody, std::shared_ptr<ForceGenerator> fg);
	void Clear();
	void UpdateForces(float deltaTime);
};