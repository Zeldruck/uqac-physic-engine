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
	std::shared_ptr<Particle> m_otherEnd;

public:
	ForceSpring(float k, float restLength, std::shared_ptr<Particle> otherEnd);

	// apply spring force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
};