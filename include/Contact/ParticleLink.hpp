#pragma once

#include "Contact/ParticleContactGenerator.hpp"

class Particle;

class ParticleLink : public ParticleContactGenerator
{
public:
	ParticleLink(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles);

	float CurrentLength() const;

	void AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit) override;

public:
	std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles;
};