#pragma once
#include "Contact/ParticleLink.hpp"

class PhysicsBody;

class ParticleRod : public ParticleLink
{
public:
	ParticleRod(std::vector<std::shared_ptr<PhysicsBody>>& particles, float length);

	void AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit) override;

public:
	float length;
};