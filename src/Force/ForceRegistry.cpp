#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"

void ForceRegistry::Add(std::shared_ptr<Particle> particle, std::shared_ptr<ForceGenerator> fg)
{
	m_registry.push_back({ particle, fg });
}

void ForceRegistry::Remove(std::shared_ptr<Particle> particle, std::shared_ptr<ForceGenerator> fg)
{
	for (auto it = m_registry.begin(); it != m_registry.end(); ++it)
	{
		if (it->particle == particle && it->forceGenerator == fg)
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
		entry.forceGenerator->UpdateForce(entry.particle, deltaTime);
	}
}