#pragma once

#include "ParticleContactGenerator.hpp";

class GroundContactGenerator : ParticleContactGenerator
{
public:
	unsigned int AddContact(ParticleContact* contact, unsigned int limit) const override;
};