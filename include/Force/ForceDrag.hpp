#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceDrag : public ForceGenerator
{
private:
	// drag coefficient
	float m_k1;		// linear drag coefficient, usually for air resistence
	float m_k2;		// quadratic drag coefficient, usually for water resistence

public:
	ForceDrag(float k1, float k2);

	// apply simplified drag force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;

	void SetDragCoefficients(float k1, float k2);
};