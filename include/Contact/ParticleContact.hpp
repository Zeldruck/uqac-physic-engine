#pragma once

#include "Vector3.hpp"
#include "Particle.hpp"
#include <memory>
#include <vector>

class ParticleContact
{
public:
	ParticleContact(std::vector<std::shared_ptr<PhysicsBody>>& particles, float restitution, float penetration, Vector3f contactNormal);

	void Resolve(float duration);
	float CalculateSeparatingVelocity();

private:
	void ResolveVelocity();
	void ResolveInterpenetration();


public:
	std::vector<std::shared_ptr<PhysicsBody>> particles;

	float restitution;
	float penetration;
	Vector3f contactNormal;

private:
	float duration;
};