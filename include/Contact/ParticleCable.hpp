#pragma once

#include "Contact/ParticleLink.hpp"

class Particle;

class ParticleCable : ParticleLink
{
public:
	ParticleCable(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles, float maxLength, float restitution);
	~ParticleCable();

	void AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit) override;

public:
	float maxLength;

	float restitution;
};