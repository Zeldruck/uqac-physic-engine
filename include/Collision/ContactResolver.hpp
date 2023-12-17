#pragma once
#include <vector>
#include <memory>

#include "Matrix3.hpp"

class Contact;
class Rigidbody;

class ContactResolver
{
public:
	ContactResolver(int iterations);

	void ResolveContacts(std::vector<std::shared_ptr<Contact>>& contacts, float duration);
	void ResolveVelocity(std::vector<std::shared_ptr<Contact>>& contacts, float duration);
	void ResolveInterpenetration(std::vector<std::shared_ptr<Contact>>& contacts, float duration);

private:
	Vector3f CalculateImpulse(std::shared_ptr<Contact>& contact, Matrix3f* inverseTensor, bool hasFriction);

private:
	int iterations;
	int iterationsUsed;

};