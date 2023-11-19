#pragma once

#include "Transform.hpp"
#include "Matrix4.hpp"
#include "Matrix3.hpp"

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
	Vector3f torque;
	float mass;
	float inverseMass;
	
	Matrix4f transformMatrix;
	Matrix3f inertiaTensor;
	Matrix3f inverseInertiaTensor;
	Matrix3f GetBoxInertiaTensorLocal();
	Matrix3f GetSphereInertiaTensorLocal();
	Matrix3f GetBoxInertiaTensorWorld();
	Matrix3f GetSphereInertiaTensorWorld();
	Matrix3f GetTriangleInertiaTensorLocal();
	Matrix3f GetTriangleInertiaTensorWorld();
	Matrix3f GetInertiaTensorWorld();
	void SetInertiaTensor(const Matrix3f& inertiaTensor);

	void ClearForce();
	void ClearTorque();
	void AddForce(const Vector3f& force);
	void RemoveForce(const Vector3f& force);
	void AddTorque(const Vector3f& torque);
	void RemoveTorque(const Vector3f& torque);
	void AddForceAtPoint(const Vector3f& force, const Vector3f& point);
	void AddForceAtBodyPoint(const Vector3f& force, const Vector3f& point);
	void RemoveForceAtPoint(const Vector3f& force, const Vector3f& point);
	void RemoveForceAtBodyPoint(const Vector3f& force, const Vector3f& point);

	void CalculateTransformMatrix();
	void CalculateInverseInertiaTensor();
	void CalculateInverseInertiaTensorWorld();
	void CalculateDerivedData();

	Vector3f const GetAcceleration();
	void SetAcceleration(const Vector3f& acceleration);
	Vector3f const GetAngularAcceleration();
	void SetAngularAcceleration(const Vector3f& angularAcceleration);

	Vector3f GetPointInWorldSpace(const Vector3f& point);

private:
	Vector3f m_acceleration;
	Vector3f m_angularAcceleration;
};