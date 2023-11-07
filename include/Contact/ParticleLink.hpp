#pragma once

#include "Contact/ParticleContactGenerator.hpp"

class Particle;

class ParticleLink : public ParticleContactGenerator
{
public:
	ParticleLink(std::vector<std::shared_ptr<Particle>>& particles);

	float CurrentLength() const;

	void virtual AddContact(std::vector<std::shared_ptr<ParticleContact>> contact, unsigned int limit);

public:
	std::vector<std::shared_ptr<Particle>> particles;
};