#pragma once

#include <vector>
#include "Particle.hpp"
#include "Vector3.hpp"

class Particle;

class EulerIntegrator
{
public:
	void Integrate(const float& deltaTime, bool isGravityEnabled = true);
	void AddParticle(std::shared_ptr<Particle> particle);
	void RemoveParticle(std::shared_ptr<Particle> particle);
	void PrintParticles();
private:
	std::vector<std::shared_ptr<Particle>> particles;
	Vector3<float> g = Vector3<float>(0.0f, -GRAVITY, 0.0f);
};