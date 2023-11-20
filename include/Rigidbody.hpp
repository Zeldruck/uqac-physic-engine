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
	TRIANGLE,
	ROD,
	RODEND,
	CYLINDER
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

	Matrix4f transformMatrix;
	Matrix3f inertiaTensor;
	Matrix3f inverseInertiaTensor;
	Matrix3f inverseInertiaTensorWorld;
	Matrix3f GetBoxInertiaTensorLocal();
	Matrix3f GetSphereInertiaTensorLocal();
	Matrix3f GetTriangleInertiaTensorLocal();
	Matrix3f GetRodInertiaTensorLocal();
	Matrix3f GetRodEndInertiaTensorLocal();
	Matrix3f GetCylinderInertiaTensorLocal();
	Matrix3f GetInverseInertiaTensorWorld();

	void ClearForce();
	void ClearTorque();
	void AddForce(const Vector3f& force);
	void AddForceAtPoint(const Vector3f& force, const Vector3f& point);
	void AddForceAtBodyPoint(const Vector3f& force, const Vector3f& point);

	void CalculateTransformMatrix();
	void CalculateDerivedData();

	Vector3f const GetAcceleration();
	Vector3f const GetAngularAcceleration();

	Vector3f GetPointInWorldSpace(const Vector3f& point);
	Vector3f GetPointInLocalSpace(const Vector3f& point);

	// WIP OR TO DELETE
	std::vector<Particle> massPoints;
	Matrix3f CalculateInertiaMatrix();
	void CalculateCenterOfMass();

private:
	Vector3f m_acceleration;
	Vector3f m_angularAcceleration;
};