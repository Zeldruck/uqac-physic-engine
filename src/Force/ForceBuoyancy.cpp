#include "Force/ForceBuoyancy.hpp"
#include "Particle.hpp"

void ForceBuoyancy::UpdateForce(Particle* particle, float deltaTime)
{
	// calculate the submersion depth
	float depth = particle->position.y;

	// check if we're out of the water
	if (depth >= m_waterHeight + m_maxDepth)
	{
		return;
	}

	Vector3f force(0, 0, 0);

	// check if we're at maximum depth
	if (depth <= m_waterHeight - m_maxDepth)
	{
		force.y = m_liquidDensity * m_volume;
		particle->AddForce(force);
		return;
	}

	// otherwise we are partly submerged
	force.y = m_liquidDensity * m_volume * (depth - m_waterHeight - m_maxDepth) / 2 * m_maxDepth;
	particle->AddForce(force);
}