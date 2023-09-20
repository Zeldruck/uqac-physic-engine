#pragma once

#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"
#include <string>

class Particle
{
public:
	Particle();
	Particle(Vector3<float> position, Vector3<float> velocity, Vector3<float> acceleration, float mass, std::string name = "Particle");
	Particle(const Particle&) = default;
	Particle(Particle&&) = default;
	~Particle() = default;

	Particle& operator=(const Particle&) = default;
	Particle& operator=(Particle&&) = default;

	std::string name;
	Vector3<float> position;
	Vector3<float> velocity;
	Vector3<float> acceleration;
	float mass = MIN_MASS;
};