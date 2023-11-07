#pragma once
#include "Contact/ParticleLink.hpp"

class Particle;

class ParticleRod : public ParticleLink
{
public:
	ParticleRod(std::vector<std::shared_ptr<Particle>>& particles, float length);

	void AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit) override;

public:
	float length;
};