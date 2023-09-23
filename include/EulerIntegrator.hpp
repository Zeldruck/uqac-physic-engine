#pragma once

#include <vector>
#include "Particle.hpp"

class Particle;

class EulerIntegrator
{
public:
	void Integrate(float deltaTime);
	void AddParticle(Particle& particle);
	void RemoveParticle(Particle& particle);
	void PrintParticles();
private:
	std::vector<Particle> particles;
};