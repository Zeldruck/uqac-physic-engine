#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceSpring : public ForceGenerator
{
private:
	// spring constant
	float m_k;
	// rest length of spring
	float m_restLength;
	// other end of spring
	Particle* m_otherEnd;

public:
	ForceSpring(float k, float restLength, Particle* otherEnd);

	// apply spring force
	void UpdateForce(Particle* particle, float deltaTime) override;
};