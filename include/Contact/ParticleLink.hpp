#pragma once

#include "Contact/ParticleContactGenerator.hpp"

class Particle;

class ParticleLink : ParticleContactGenerator
{
public:
	ParticleLink(Particle* particles[2]);

	float CurrentLength() const;

	unsigned int AddContact(ParticleContact* contact, unsigned int limit) override = 0;

public:
	Particle* particles[2];
};