#pragma once
#include <vector>
#include <memory>

class Contact;
class Primitive;
class Sphere;

class ContactGenerator
{
public:
	ContactGenerator(float maxContacts);

	void DetectContacts(const Primitive& primitiveA, const Primitive& primitiveB);

private:
	void DetectSandS(const Sphere& sphereA, const Sphere& sphereB);

private:
	std::vector<std::shared_ptr<Contact>> contacts;

	unsigned int maxContacts;
	unsigned int currentContacts;
};