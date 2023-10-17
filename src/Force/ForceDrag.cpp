#include "Force/ForceDrag.hpp"
#include "Particle.hpp"

ForceDrag::ForceDrag(float k1, float k2) : 
	m_k1(k1), 
	m_k2(k2) 
{
};

void ForceDrag::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	// calculate the total drag coefficient
	float dragCoeff = particle->velocity.GetLength();
	dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

	// calculate the final force and apply it
	Vector3f finalForce = particle->velocity;
	finalForce.Normalize();
	finalForce *= -dragCoeff;
	particle->AddForce(finalForce);
}