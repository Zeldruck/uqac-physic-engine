#pragma once

class ParticleContact;

class ParticleContactGenerator
{
public:
	virtual unsigned int AddContact(ParticleContact* contact, unsigned int limit) const = 0;
};