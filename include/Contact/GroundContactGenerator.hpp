#pragma once

#include "ParticleContactGenerator.hpp";

class GroundContactGenerator : ParticleContactGenerator
{
public:
	unsigned int AddContact(std::shared_ptr<ParticleContact> contact, unsigned int limit) const override;
};