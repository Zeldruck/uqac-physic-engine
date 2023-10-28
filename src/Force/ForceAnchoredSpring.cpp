#include "Force/ForceAnchoredSpring.hpp"
#include "Particle.hpp"

ForceAnchoredSpring::ForceAnchoredSpring(float k, float restLength, Vector3f anchor) :
	m_k(k),
	m_restLength(restLength),
	m_anchor(anchor)
{
}

void ForceAnchoredSpring::UpdateForce(std::shared_ptr<PhysicsBody> physicBody, float deltaTime)
{
	if (physicBody->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = physicBody->GetPosition() - m_anchor;

	if (springVector.x == 0 && springVector.y == 0 && springVector.z == 0)
		return;

	// calculate the magnitude of the force
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	physicBody->AddForce(force);
}

void ForceAnchoredSpring::SetAnchor(Vector3f anchor)
{
	m_anchor = anchor;
}

void ForceAnchoredSpring::SetSpringConstant(float k)
{
	m_k = k;
}