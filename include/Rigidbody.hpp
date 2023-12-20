#pragma once

#include <vector>
#include <string>
#include "Matrix3.hpp"
#include "Matrix4.hpp"
#include "Quaternion.hpp"

class BoundingSphere;
class BoundingBox;

enum RigidbodyType
{
	CUBE,
	SPHERE,
	TETRAHEDRON
};

class Rigidbody
{
public:
	Rigidbody();
	Rigidbody(std::string name);
	Rigidbody(std::string name, Vector3f position);
	Rigidbody(std::string name, Vector3f position, float mass);
	Rigidbody(std::string name, RigidbodyType type);
	Rigidbody(std::string name, RigidbodyType type, Vector3f position);
	Rigidbody(std::string name, RigidbodyType type, Vector3f position, float mass);
	Rigidbody(std::string name, RigidbodyType type, Vector3f position, Quaternionf rotation, float mass);
	Rigidbody(std::string name, RigidbodyType type, Vector3f position, Quaternionf rotation, Vector3f scale, float mass);

	bool isAwake;
	

	std::string name;
	RigidbodyType type;
	Vector3f position;
	Quaternionf rotation;
	Vector3f scale;
	Vector3f velocity;
	Vector3f force;
	Vector3f angularVelocity;
	Vector3f torque;

	Vector3f centerOfMass;
	float mass;
	float inverseMass;

	Matrix4f transformMatrix;
	Matrix3f inertiaTensor;
	Matrix3f inverseInertiaTensor;
	Matrix3f inverseInertiaTensorWorld;

	Matrix3f GetInverseInertiaTensorWorld();
	Matrix3f GetBoxInertiaTensorLocal();
	Matrix3f GetSphereInertiaTensorLocal();
	Matrix3f GetTetrahedronInertiaTensorLocal();

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

	std::shared_ptr<BoundingSphere> GetBoundingSphere();
	BoundingBox GetBoundingBox();
	std::shared_ptr<BoundingSphere> m_boundingSphere;

private:
	Vector3f m_acceleration;
	Vector3f m_angularAcceleration;

};