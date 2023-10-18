#pragma once

#include "Contact/ParticleContactGenerator.hpp"

class Particle;

class ParticleLink : ParticleContactGenerator
{
public:
	float CurrentLength() const;

	unsigned int AddContact(ParticleContact* contact, unsigned int limit) const override = 0;

public:
	Particle* particles[2];
};