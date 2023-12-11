#pragma once
#include <memory>
#include <vector>
#include "Vector3.hpp"

class Rigidbody;

class Contact
{
public:
	Contact() = default;
	Contact(std::vector<std::shared_ptr<Rigidbody>>& particles, Vector3f contactPoint, Vector3f contactNormal, float penetration);

public:
	std::vector<std::shared_ptr<Rigidbody>> particles;

	Vector3f contactPoint;
	Vector3f contactNormal;

	float penetration;
};