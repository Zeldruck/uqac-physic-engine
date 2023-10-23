#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceAnchoredSpring : public ForceGenerator
{
private:
	Vector3f m_anchor;
	float m_k;
	float m_restLength;
	 
public:
	ForceAnchoredSpring(float k, float restLength, Vector3f anchor);

	// apply spring force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
	void SetAnchor(Vector3f anchor);
	void SetSpringConstant(float k);
};