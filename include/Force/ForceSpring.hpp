#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceSpring : public ForceGenerator
{
private:
	// spring constant
	float m_k;
	// rest length of spring
	float m_restLength;
	// other end of spring particle
	std::shared_ptr<Particle> m_otherParticle;
	// other end of spring rigidbody
	std::shared_ptr<Rigidbody> m_otherRigidbody;

	Vector3f connectionPoint;
	Vector3f otherConnectionPoint;

public:
	ForceSpring(std::shared_ptr<Particle> otherEnd, float k, float restLength);
	ForceSpring(std::shared_ptr<Rigidbody> otherEnd, float k, float restLength);
	ForceSpring(std::shared_ptr<Rigidbody> otherEnd, Vector3f connectionPoint, Vector3f otherConnectionPoint, float k, float restength);

	// apply spring force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
	void UpdateForce(std::shared_ptr<Rigidbody> rigidBody, float deltaTime) override;

	void SetOtherEnd(std::shared_ptr<Particle> otherEnd);
	void SetSpringConstant(float k);
};