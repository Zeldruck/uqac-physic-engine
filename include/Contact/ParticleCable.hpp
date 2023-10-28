#pragma once

#include "Contact/ParticleLink.hpp"

class PhysicsBody;

class ParticleCable : public ParticleLink
{
public:
	ParticleCable(std::vector<std::shared_ptr<PhysicsBody>>& particles, float maxLength, float restitution);

	void AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit) override;

public:
	float maxLength;

	float restitution;
};