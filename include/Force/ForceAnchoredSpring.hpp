#pragma once

#include "ForceGenerator.hpp"
#include "Vector3.hpp"

class ForceAnchoredSpring : public ForceGenerator
{
private:
	Vector3f m_anchor;
	float m_k;
	float m_restLength;

	Vector3f connectionPoint;
	 
public:
	ForceAnchoredSpring(Vector3f anchor, float k, float restLength);
	ForceAnchoredSpring(float k, float restLength, Vector3f anchor, Vector3f connectionPoint);

	// apply spring force
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
	void UpdateForce(std::shared_ptr<Rigidbody> rigidBody, float deltaTime) override;
	void SetAnchor(Vector3f anchor);
	Vector3f GetAnchor();
	void SetSpringConstant(float k);
};