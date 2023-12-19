#include "Force/ForceBuoyancy.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceBuoyancy::ForceBuoyancy(float maxDepth, float volume, float waterHeight, float liquidDensity) :
	m_maxDepth(maxDepth),
	m_volume(volume),
	m_waterHeight(waterHeight),
	m_liquidDensity(liquidDensity)
{
}

void ForceBuoyancy::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
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

void ForceBuoyancy::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	// calculate the submersion depth
	float depth = rigidbody->position.y;

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
		rigidbody->AddForce(force);
		return;
	}

	// otherwise we are partly submerged
	force.y = m_liquidDensity * m_volume * (depth - m_waterHeight - m_maxDepth) / 2 * m_maxDepth;
	rigidbody->AddForce(force);
}