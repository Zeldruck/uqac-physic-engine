#pragma once
#include "Contact/ParticleLink.hpp"

class Particle;

class ParticleRod : public ParticleLink
{
public:
	ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles, float length);

	void AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit) override;

public:
	float length;
};