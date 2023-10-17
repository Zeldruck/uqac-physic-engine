#pragma once

#include <memory>

class Particle;

class ForceGenerator
{
public:
	virtual void UpdateForce(Particle* particle, float duration) = 0;
}; 