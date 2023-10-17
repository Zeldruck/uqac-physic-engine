#pragma once

#include <memory>

class Particle;

class ForceGenerator
{
public:
	virtual void UpdateForce(std::shared_ptr<Particle> particle, float duration) = 0;
}; 