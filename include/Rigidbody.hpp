#pragma once

#include "Transform.hpp"
#include "Matrix4.hpp"
#include "Matrix3.hpp"
#include <vector>

class Particle;

// TO REMOVE LATER
enum RigidbodyType
{
	BOX,
	SPHERE,
	TRIANGLE
};

class Rigidbody
{
public:
	Rigidbody();
	Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name, RigidbodyType type, std::vector<Particle> massPoints = std::vector<Particle>(), float linearDamping = 0.0f, float angularDamping = 0.5f);

	std::string name;
	RigidbodyType type;
	Transform transform;
	Vector3f velocity;
	Vector3f force;
	Vector3f angularVelocity;
	Vector3f torque;
	Vector3f centerOfMass;
	float mass;
	float inverseMass;
	float linearDamping;
	float angularDamping;
	
	std::vector<Particle> massPoints;
	void CalculateInertiaMatrix();
	void CalculateCenterOfMass();

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
	void AddForceAtPoint(const Vector3f& force, const Vector3f& point);
	void AddForceAtBodyPoint(const Vector3f& force, const Vector3f& point);
	void RemoveForceAtPoint(const Vector3f& force, const Vector3f& point);
	void RemoveForceAtBodyPoint(const Vector3f& force, const Vector3f& point);

	void CalculateTransformMatrix();
	void CalculateInverseInertiaTensor();
	void CalculateInverseInertiaTensorWorld();
	void CalculateDerivedData();

	Vector3f const GetAcceleration();
	void SetAcceleration(const Vector3f& acceleration);	// for testing purpose
	Vector3f const GetAngularAcceleration();
	void SetAngularAcceleration(const Vector3f& angularAcceleration); // for testing purpose

	Vector3f GetPointInWorldSpace(const Vector3f& point);

private:
	Vector3f m_acceleration;
	Vector3f m_angularAcceleration;
};