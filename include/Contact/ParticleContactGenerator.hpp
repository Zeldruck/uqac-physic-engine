#pragma once
#include <vector>
#include <memory>

class ParticleContact;

class ParticleContactGenerator
{
public:
	virtual void AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit) = 0;
};