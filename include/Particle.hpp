#pragma once

#include <string>
#include "Vector3.hpp"
#include "PhysicsBody.hpp"

class Particle : public PhysicsBody
{
public:
	Particle();
	Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name = "Particle");
	Particle(const Particle&) = default;
	Particle(Particle&&) = default;
	~Particle() = default;

	Particle& operator=(const Particle&) = default;
	Particle& operator=(Particle&&) = default;

	Vector3f position;

	Vector3f GetPosition() const override;
	void SetPosition(Vector3f newPosition) override;
};