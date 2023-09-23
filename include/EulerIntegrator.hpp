#pragma once

#include <vector>
#include "Particle.hpp"

class Particle;

class EulerIntegrator
{
public:
	void Integrate(float deltaTime);
	void AddParticle(std::shared_ptr<Particle> particle);
	void RemoveParticle(std::shared_ptr<Particle> particle);
	void PrintParticles();
private:
	std::vector<std::shared_ptr<Particle>> particles;
};