#pragma once

#include "Contact/ParticleLink.hpp"

class Particle;
class Rigidbody;

class ParticleCable : public ParticleLink
{
public:
	ParticleCable(std::vector<std::shared_ptr<Particle>>& particles, float maxLength, float restitution);
	void AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit) override;

public:
	float maxLength;

	float restitution;
};