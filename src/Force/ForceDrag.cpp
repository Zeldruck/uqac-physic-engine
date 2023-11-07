#include "Force/ForceDrag.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceDrag::ForceDrag(float k1, float k2) : 
	m_k1(k1), 
	m_k2(k2) 
{
};

void ForceDrag::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	float velocityLength = particle->velocity.GetLength();
	if (velocityLength < 0.001f)
		return;

	if (particle->mass < 0.001f)
		return;
	
	// calculate the total drag coefficient
	float dragCoeff = velocityLength;
	dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

	// calculate the force and apply it
	Vector3f force = particle->velocity.GetNormalized() * -dragCoeff;
	particle->AddForce(force);
}

void ForceDrag::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	float velocityLength = rigidbody->velocity.GetLength();
	if (velocityLength < 0.001f)
		return;

	if (rigidbody->mass < 0.001f)
		return;

	// calculate the total drag coefficient
	float dragCoeff = velocityLength;
	dragCoeff = m_k1 * dragCoeff + m_k2 * dragCoeff * dragCoeff;

	// calculate the force and apply it
	Vector3f force = rigidbody->velocity.GetNormalized() * -dragCoeff;
	rigidbody->AddForce(force);
}

void ForceDrag::SetDragCoefficients(float k1, float k2)
{
	m_k1 = k1;
	m_k2 = k2;
}