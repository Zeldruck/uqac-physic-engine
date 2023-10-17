#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceAnchoredSpring : public ForceGenerator
{
private:
	Vector3<float> m_anchor;
	float m_springConstant;
	float m_restLength;
	 
public:
	// apply spring force
	void UpdateForce(Particle* particle, float deltaTime) override;
};