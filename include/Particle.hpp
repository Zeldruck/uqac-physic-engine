#pragma once

#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"
#include <string>

class Particle
{
public:
	Particle();
	Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name = "Particle");
	Particle(const Particle&) = default;
	Particle(Particle&&) = default;
	~Particle() = default;

	Particle& operator=(const Particle&) = default;
	Particle& operator=(Particle&&) = default;

	std::string name;
	Vector3f position;
	Vector3f velocity;
	Vector3f acceleration;
	float mass = MIN_MASS;
};