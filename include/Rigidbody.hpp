#pragma once

#include "PhysicsBody.hpp"
#include "Transform.hpp"

class Rigidbody : public PhysicsBody
{
public:
	Rigidbody();
	Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name);

	Transform transform;
	Vector3f angularVelocity;
	Vector3f angularAcceleration;
	Vector3f torque;
	Vector3f momentOfInertia;

	Vector3f GetPosition() const override;
};