#pragma once

#include "Contact/ParticleContactGenerator.hpp"

class Particle;

class ParticleLink : public ParticleContactGenerator
{
public:
	ParticleLink(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles);

	float CurrentLength() const;

	void virtual AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit);

public:
	std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles;
};