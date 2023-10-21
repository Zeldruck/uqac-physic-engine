#pragma once

#include "Vector3.hpp"
#include "Particle.hpp"
#include <memory>
#include <vector>

class ParticleContact
{
public:
	ParticleContact(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles, float restitution, float penetration, Vector3f contactNormal);

	void Resolve(float duration);
	float CalculateSeparatingVelocity();

private:
	void ResolveVelocity();
	void ResolveInterpenetration();


public:
	std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles;

	float restitution;
	float penetration;
	Vector3f contactNormal;
};