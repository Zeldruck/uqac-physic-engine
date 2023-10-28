#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"
#include "PhysicsBody.hpp"

void ForceRegistry::Add(std::shared_ptr<PhysicsBody> physicBody, std::shared_ptr<ForceGenerator> fg)
{
	m_registry.push_back({ physicBody, fg });
}

void ForceRegistry::Remove(std::shared_ptr<PhysicsBody> physicBody, std::shared_ptr<ForceGenerator> fg)
{
	for (auto it = m_registry.begin(); it != m_registry.end(); ++it)
	{
		if (it->physicBody == physicBody && it->forceGenerator == fg)
		{
			m_registry.erase(it);
			break;
		}
	}
}

void ForceRegistry::Clear()
{
	m_registry.clear();
}

void ForceRegistry::UpdateForces(float deltaTime)
{
	for (auto& entry : m_registry)
	{
		entry.forceGenerator->UpdateForce(entry.physicBody, deltaTime);
	}
}