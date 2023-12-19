#include "Force/ForceAnchoredSpring.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceAnchoredSpring::ForceAnchoredSpring(Vector3f anchor, float k, float restLength) :
	m_k(k),
	m_restLength(restLength),
	m_anchor(anchor),
	connectionPoint(Vector3f::Zero)
{
}

ForceAnchoredSpring::ForceAnchoredSpring(Vector3f anchor, Vector3f connectionPoint, float k, float restLength) :
	m_k(k),
	m_restLength(restLength),
	m_anchor(anchor),
	connectionPoint(connectionPoint)
{}

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

	// calculate local to world space point
	Vector3f localInWorldSpace = rigidbody->GetPointInWorldSpace(connectionPoint);

	// calculate the force of the spring
	Vector3f force = localInWorldSpace - m_anchor;

	// calculate the magnitude of the spring
	float magnitude = force.GetLength();
	magnitude = std::abs(magnitude - m_restLength);
	magnitude *= m_k;

	// calculate final force and apply it
	force.Normalize();
	force *= -magnitude;

	rigidbody->AddForceAtPoint(force, localInWorldSpace);
}

void ForceAnchoredSpring::SetAnchor(Vector3f anchor)
{
	m_anchor = anchor;
}

Vector3f ForceAnchoredSpring::GetAnchor()
{
	return m_anchor;
}

void ForceAnchoredSpring::SetSpringConstant(float k)
{
	m_k = k;
}