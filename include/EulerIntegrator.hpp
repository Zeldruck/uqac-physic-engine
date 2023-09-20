#pragma once

class Particle;

class EulerIntegrator
{
public:
	void Integrate(Particle& particle, float deltaTime);
};