#include "Force/ForceAnchoredSpring.hpp"
#include "Particle.hpp"

ForceAnchoredSpring::ForceAnchoredSpring(float k, float restLength, Vector3f anchor) :
	m_k(k),
	m_restLength(restLength),
	m_anchor(anchor)
{
}

void ForceAnchoredSpring::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	// calculate the vector of the spring
	Vector3f springVector = particle->position - m_anchor;

	// calculate the magnitude of the force
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	particle->AddForce(force);
}

void ForceAnchoredSpring::SetAnchor(Vector3f anchor)
{
	m_anchor = anchor;
}