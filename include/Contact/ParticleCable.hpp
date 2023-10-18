#pragma once

#include "Contact/ParticleLink.hpp"

class Particle;

class ParticleCable : ParticleLink
{
public:
	ParticleCable(Particle* particles[2], float maxLength, float restitution);
	~ParticleCable();

	unsigned int AddContact(ParticleContact* contact, unsigned int limit) override;

public:
	float maxLength;

	float restitution;
};