#pragma once
#include <vector>
#include <memory>

#include "Vector3.hpp"

class Contact;
class Primitive;
class Sphere;
class Plane;
class Box;

class ContactGenerator
{
public:
	ContactGenerator(float maxContacts);

private:
	void DetectSandS(const Sphere& sphereA, const Sphere& sphereB);
	void DetectSandHS(const Sphere& sphere, const Plane& plane);
	void DetectSandP(const Sphere& sphere, const Plane& plane);
	void DetectSandB(const Sphere& sphere, const Box& box);

	void DetectBandP(const Box& box, const Plane& plane);
	void DetectBandB(const Box& boxA, const Box& boxB);

	bool SAT(const Box& boxA, const Box& boxB, const Vector3f& axis);
	bool SATBandB(const Box& boxA, const Box& boxB);
	float AxisPenetrationBandB(float boxAProjection, float boxBProjection, const Vector3f& center, const Vector3f& axis);

private:
	std::vector<std::shared_ptr<Contact>> contacts;

	unsigned int maxContacts;
	unsigned int currentContacts;
};