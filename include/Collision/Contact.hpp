#pragma once
#include <memory>
#include <vector>
#include "Vector3.hpp"
#include "Matrix3.hpp"

class Rigidbody;

class Contact
{
public:
	Contact() = default;
	Contact(std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, Vector3f contactPoint, Vector3f contactNormal, float penetration);

	void PreCalculation(float duration);
	void CalculateContactBasis();
	void CalculateDeltaVelocity(float duration);
	Vector3f CalculateLocalVelocity(int index, float duration);

public:
	std::vector<std::shared_ptr<Rigidbody>> rigidbodies;

	Vector3f contactPoint;
	Vector3f contactNormal;

	float penetration;

	float deltaVelocity;

	Matrix3f contactToWorld;
	Vector3f contactVelocity;

	Vector3f relativeContactPosition[2];
};