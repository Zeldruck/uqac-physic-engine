#pragma once
#include "Contact/ParticleLink.hpp"

class Particle;

class ParticleRod : ParticleLink
{
public:
	ParticleRod(Particle* particles[2], float length);
	~ParticleRod();

	unsigned int AddContact(ParticleContact* contact, unsigned int limit) const override;

public:
	float length;
};