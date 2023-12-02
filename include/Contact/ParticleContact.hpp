#pragma once

#include "Vector3.hpp"
#include "Particle.hpp"
#include <memory>
#include <vector>

class ParticleContact
{
public:
	ParticleContact(std::vector<std::shared_ptr<Particle>>& particles, float restitution, float penetration, Vector3f contactNormal);

	void Resolve(float duration);
	float CalculateSeparatingVelocity();

private:
	void ResolveVelocity(float duration);
	void ResolveInterpenetration(float duration);


public:
	std::vector<std::shared_ptr<Particle>> particles;

	float restitution;
	float penetration;
	Vector3f contactNormal;
};