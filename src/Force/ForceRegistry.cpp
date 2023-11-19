#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"
#include "Particle.hpp"

void ForceRegistry::Add(std::shared_ptr<Particle> particle, std::shared_ptr<ForceGenerator> fg)
{
	m_registry.push_back({ particle, fg });
}

void ForceRegistry::Add(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<ForceGenerator> fg)
{
	m_registryRigidbody.push_back({ rigidbody, fg });
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

void ForceRegistry::Remove(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<ForceGenerator> fg)
{
	for (auto it = m_registryRigidbody.begin(); it != m_registryRigidbody.end(); ++it)
	{
		if (it->rigidbody == rigidbody && it->forceGenerator == fg)
		{
			m_registryRigidbody.erase(it);
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

	for (auto& entry : m_registryRigidbody)
	{
		entry.forceGenerator->UpdateForce(entry.rigidbody, deltaTime);
	}
}