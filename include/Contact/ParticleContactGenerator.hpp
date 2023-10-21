#pragma once
#include <vector>
#include <memory>

class ParticleContact;

class ParticleContactGenerator
{
public:
	virtual void AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit);
};