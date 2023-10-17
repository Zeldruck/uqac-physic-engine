#pragma once
#include <vector>

class ForceGenerator;
class Particle;

class ForceRegistry
{
private:
	struct ForceEntry
	{
		Particle* particle;
		ForceGenerator* forceGenerator;
	};

	using Registry = std::vector<ForceEntry>;
	Registry m_registry;

public:
	void Add(Particle* particle, ForceGenerator* fg);
	void Remove(Particle* particle, ForceGenerator* fg);
	void Clear();
	void UpdateForces(float deltaTime);
};