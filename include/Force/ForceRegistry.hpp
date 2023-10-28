#pragma once
#include <vector>
#include <memory>

class ForceGenerator;
class PhysicsBody;

class ForceRegistry
{
private:
	struct ForceEntry
	{
		std::shared_ptr<PhysicsBody> physicBody;
		std::shared_ptr<ForceGenerator> forceGenerator;
	};

	using Registry = std::vector<ForceEntry>;
	Registry m_registry;

public:
	void Add(std::shared_ptr<PhysicsBody> physicBody, std::shared_ptr<ForceGenerator> fg);
	void Remove(std::shared_ptr<PhysicsBody> physicBody, std::shared_ptr<ForceGenerator> fg);
	void Clear();
	void UpdateForces(float deltaTime);
};