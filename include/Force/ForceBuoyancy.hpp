#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceBuoyancy : public ForceGenerator
{
private:
	// particle property
	float m_maxDepth;
	float m_volume;

	// liquid property
	float m_waterHeight;
	float m_liquidDensity;

public:
	ForceBuoyancy(float maxDepth, float volume, float waterHeight, float liquidDensity);

	// apply buoyancy force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
	void UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime) override;
};