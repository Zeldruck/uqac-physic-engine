#include "Force/ForceAnchoredSpring.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceAnchoredSpring::ForceAnchoredSpring(float k, float restLength, Vector3f anchor) :
	m_k(k),
	m_restLength(restLength),
	m_anchor(anchor)
{
}

void ForceAnchoredSpring::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	if (particle->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = particle->position - m_anchor;

	if (springVector.x == 0 && springVector.y == 0 && springVector.z == 0)
		return;

	// calculate the magnitude of the force
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	particle->AddForce(force);
}

void ForceAnchoredSpring::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	if (rigidbody->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = rigidbody->transform.position - m_anchor;

	if (springVector.x == 0 && springVector.y == 0 && springVector.z == 0)
		return;

	// calculate the magnitude of the force
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	rigidbody->AddForce(force);
}

void ForceAnchoredSpring::SetAnchor(Vector3f anchor)
{
	m_anchor = anchor;
}

void ForceAnchoredSpring::SetSpringConstant(float k)
{
	m_k = k;
}