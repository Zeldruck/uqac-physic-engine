#include "Force/ForceSpring.hpp"
#include "Particle.hpp"

ForceSpring::ForceSpring(float k, float restLength, Particle* otherEnd) : 
	m_k(k), 
	m_restLength(restLength), 
	m_otherEnd(otherEnd) 
{
}

void ForceSpring::UpdateForce(Particle* particle, float deltaTime)
{
	// calculate the vector of the spring
	Vector3f springVector = particle->position - m_otherEnd->position;

	// calculate the magnitude of the spring
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	particle->AddForce(force);
}