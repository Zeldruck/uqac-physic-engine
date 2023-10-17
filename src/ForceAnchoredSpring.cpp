#include "ForceAnchoredSpring.hpp"
#include "Particle.hpp"

void ForceAnchoredSpring::UpdateForce(Particle* particle, float deltaTime)
{
	// calculate the vector of the spring
	Vector3f springVector = particle->position - m_anchor;

	// calculate the magnitude of the force
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_springConstant * (magnitude - m_restLength) * springVector.GetNormalized();
	particle->AddForce(force);
}