#pragma once

#include <string>
#include "Vector3.hpp"

class PhysicsBody
{
public:
	PhysicsBody();
	PhysicsBody(std::string name);
	PhysicsBody(Vector3f velocity, Vector3f acceleration, float mass, std::string name);

	std::string name;

	float mass;
	Vector3f velocity;
	Vector3f force;

	void ClearForce();
	void AddForce(const Vector3f& force);
	void RemoveForce(const Vector3f& force);
	Vector3f const GetAcceleration();

	virtual Vector3f GetPosition() const;	

protected:
	Vector3f acceleration;
};