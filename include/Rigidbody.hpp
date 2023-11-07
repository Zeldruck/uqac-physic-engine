#pragma once

#include "Transform.hpp"

class Rigidbody
{
public:
	Rigidbody();
	Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name);

	std::string name;
	Transform transform;
	Vector3f velocity;
	Vector3f force;
	Vector3f angularVelocity;
	Vector3f momentOfInertia;
	Vector3f torque;
	float mass;

	void ClearForce();
	void AddForce(const Vector3f& force);
	void RemoveForce(const Vector3f& force);
	void AddTorque(const Vector3f& torque);
	void RemoveTorque(const Vector3f& torque);

	Vector3f const GetAcceleration();
	void SetAcceleration(const Vector3f& acceleration);
	Vector3f const GetAngularAcceleration();
	void SetAngularAcceleration(const Vector3f& angularAcceleration);

private:
	Vector3f m_acceleration;
	Vector3f m_angularAcceleration;
};