#pragma once
#include <vector>
#include <memory>

class Contact;
class Primitive;
class Sphere;
class Plane;
class Box;

class ContactGenerator
{
public:
	ContactGenerator(float maxContacts);

	void DetectContacts(const Primitive& primitiveA, const Primitive& primitiveB);

private:
	void DetectSandS(const Sphere& sphereA, const Sphere& sphereB);
	void DetectSandHS(const Sphere& sphere, const Plane& plane);
	void DetectSandP(const Sphere& sphere, const Plane& plane);
	void DetectSandB(const Sphere& sphere, const Box& box);

	void DetectBandP(const Box& box, const Plane& plane);

private:
	std::vector<std::shared_ptr<Contact>> contacts;

	unsigned int maxContacts;
	unsigned int currentContacts;
};