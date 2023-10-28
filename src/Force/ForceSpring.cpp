#include "Force/ForceSpring.hpp"
#include "Particle.hpp"

ForceSpring::ForceSpring(float k, float restLength, std::shared_ptr<PhysicsBody> otherEnd) :
	m_k(k), 
	m_restLength(restLength), 
	m_otherEnd(otherEnd) 
{
}

void ForceSpring::UpdateForce(std::shared_ptr<PhysicsBody> physicBody, float deltaTime)
{
	if (physicBody->mass < 1.0f)
		return;

	// calculate the vector of the spring
	Vector3f springVector = m_otherEnd->GetPosition() - physicBody->GetPosition();

	// calculate the magnitude of the spring
	float magnitude = springVector.GetLength();

	// calculate the final force and apply it
	Vector3f force = -m_k * (magnitude - m_restLength) * springVector.GetNormalized();
	physicBody->AddForce(force);
}

void ForceSpring::SetOtherEnd(std::shared_ptr<PhysicsBody> otherEnd)
{
	m_otherEnd = otherEnd;
}

void ForceSpring::SetSpringConstant(float k)
{
	m_k = k;
}