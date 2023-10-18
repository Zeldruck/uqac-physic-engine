#pragma once

#include "Vector3.hpp"
#include "Particle.hpp"

class ParticleContact
{
public:
	ParticleContact(Particle* particles[2], float restitution, float penetration, Vector3f contactNormal);
	~ParticleContact();

	void Resolve(float duration);
	float CalculateSeparatingVelocity();

private:
	void ResolveVelocity();
	void ResolveInterpenetration();


public:
	Particle* particles[2];

	float restitution;
	float penetration;
	Vector3f contactNormal;
};