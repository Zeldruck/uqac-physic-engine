#pragma once

#include <string>
#include "Vector3.hpp"

class Particle
{
public:
	Particle();
	Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name);
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

	void ClearForce();
	void AddForce(const Vector3f& force);
	void RemoveForce(const Vector3f& force);
	Vector3f const GetAcceleration();
	Vector3f SetAcceleration(const Vector3f& acceleration);

private:
	Vector3f m_acceleration;
};