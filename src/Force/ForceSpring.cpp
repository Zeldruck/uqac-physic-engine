#include "Force/ForceSpring.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceSpring::ForceSpring(std::shared_ptr<Particle> otherEnd, float k, float restLength) :
	m_k(k),
	m_restLength(restLength),
	m_otherParticle(otherEnd),
	m_otherRigidbody(nullptr),
	connectionPoint(Vector3f::Zero),
	otherConnectionPoint(Vector3f::Zero)
{
}

ForceSpring::ForceSpring(std::shared_ptr<Rigidbody> otherEnd, float k, float restLength) :
	m_k(k),
	m_restLength(restLength),
	m_otherParticle(nullptr),
	m_otherRigidbody(otherEnd),
	connectionPoint(Vector3f::Zero),
	otherConnectionPoint(Vector3f::Zero)
{
}

ForceSpring::ForceSpring(std::shared_ptr<Rigidbody> otherEnd, Vector3f connectionPoint, Vector3f otherConnectionPoint, float k, float restLength) :
	m_k(k),
	m_restLength(restLength),
	m_otherParticle(nullptr),
	m_otherRigidbody(otherEnd),
	connectionPoint(connectionPoint),
	otherConnectionPoint(otherConnectionPoint)
{
}

void ForceSpring::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	if (particle->mass < 1.0f)
		return;

	// Calculate the vector of the spring
	Vector3f force = m_otherParticle->position - particle->position;

	// Calculate the magnitude of the spring
	float displacement = force.GetLength() - m_restLength;
	float magnitude = m_k * displacement;

	// Calculate the final force and apply it
	force.Normalize();
	force *= -magnitude;

	// Apply the force to both particles
	particle->AddForce(force);
	m_otherParticle->AddForce(force.GetInvert());  // Opposite force applied to the other particle
}

void ForceSpring::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	if (rigidbody->mass < 1.0f)
		return;

	// Calculate local to world space points
	Vector3f localInWorldSpace = rigidbody->GetPointInWorldSpace(connectionPoint);
	Vector3f otherInWorldSpace = rigidbody->GetPointInWorldSpace(otherConnectionPoint);

	// Calculate the force of the spring
	Vector3f force = localInWorldSpace - otherInWorldSpace;

	// Calculate the displacement from the rest length
	float displacement = force.GetLength() - m_restLength;

	// Calculate the magnitude of the spring force
	float magnitude = m_k * displacement;

	// Calculate the final force and apply it to both connection points
	force.Normalize();
	force *= -magnitude;

	rigidbody->AddForceAtPoint(force, localInWorldSpace);
	rigidbody->AddForceAtPoint(force.GetInvert(), otherInWorldSpace); // Opposite force applied to the other connection point
}

void ForceSpring::SetOtherEnd(std::shared_ptr<Particle> otherEnd)
{
	m_otherParticle = otherEnd;
}

void ForceSpring::SetSpringConstant(float k)
{
	m_k = k;
}