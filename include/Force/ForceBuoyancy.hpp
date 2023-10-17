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
	// apply buoyancy force
	void UpdateForce(Particle* particle, float deltaTime) override;
};