#pragma once

#include <string>
#include "Vector3.hpp"

class Particle
{
public:
	Particle();
	Particle(std::string name);
	Particle(std::string name, Vector3f position);
	Particle(std::string name, Vector3f position, float mass);
	Particle(std::string name, Vector3f position, float mass, Vector3f velocity);
	Particle(std::string name, Vector3f position, float mass, Vector3f velocity, Vector3f acceleration);
	Particle(const Particle&) = default;
	Particle(Particle&&) = default;
	~Particle() = default;

	Particle& operator=(const Particle&) = default;
	Particle& operator=(Particle&&) = default;

	std::string name;
	Vector3f position;
	Vector3f velocity;
	Vector3f force;
	float mass;

	void AddForce(const Vector3f& force);
	void ClearForce();

	Vector3f SetAcceleration(const Vector3f& acceleration);
	Vector3f const GetAcceleration();

private:
	Vector3f m_acceleration;
};