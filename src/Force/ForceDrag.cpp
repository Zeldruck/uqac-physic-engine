#include "Force/ForceDrag.hpp"
#include "Particle.hpp"

ForceDrag::ForceDrag(float k1, float k2) : 
	m_k1(k1), 
	m_k2(k2) 
{
};

void ForceDrag::UpdateForce(std::shared_ptr<PhysicsBody> particle, float deltaTime)
{
	if (particle->velocity.GetLength() < 0.001f)
		return;

	if (particle->mass < 0.001f)
		return;
	
	// calculate the total drag coefficient
	float dragCoeff = particle->velocity.GetLength();
	dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

	// calculate the force and apply it
	Vector3f force = particle->velocity.GetNormalized() * -dragCoeff;
	particle->AddForce(force);
}

void ForceDrag::SetDragCoefficients(float k1, float k2)
{
	m_k1 = k1;
	m_k2 = k2;
}