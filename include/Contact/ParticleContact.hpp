#pragma once

#include "Vector3.hpp"
#include "Particle.hpp"

class ParticleContact
{
public:
	ParticleContact(Particle* particles, float restitution);
	~ParticleContact();

	void Resolve(float duration);
	void CalculateSeparatingVelocity();

private:
	void ResolveVelocity();
	void ResolveInterpenetration();


public:
	Particle* particle[2];

	float restitution;
	float penetration;
	Vector3f contactNormal;
};