#include "Force/ForceSpring.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

ForceSpring::ForceSpring(float k, float restLength, std::shared_ptr<Particle> otherEnd) :
	m_k(k), 
	m_restLength(restLength), 
	m_otherParticle(otherEnd), 
	m_otherRigidbody(nullptr)
{
}

ForceSpring::ForceSpring(float k, float restLength, std::shared_ptr<Rigidbody> otherEnd) :
	m_k(k),
	m_restLength(restLength),
	m_otherParticle(nullptr),
	m_otherRigidbody(otherEnd)
{
}

void ForceSpring::UpdateForce(std::shared_ptr<Particle> particle, float deltaTime)
{
	if (particle->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = m_otherParticle->position - particle->position;

	// calculate the magnitude of the spring
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	particle->AddForce(force);
}

void ForceSpring::UpdateForce(std::shared_ptr<Rigidbody> rigidbody, float deltaTime)
{
	if (rigidbody->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = m_otherRigidbody->transform.position - rigidbody->transform.position;

	// calculate the magnitude of the spring
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	rigidbody->AddForce(force);
}

void ForceSpring::SetOtherEnd(std::shared_ptr<Particle> otherEnd)
{
	m_otherParticle = otherEnd;
}

void ForceSpring::SetSpringConstant(float k)
{
	m_k = k;
}