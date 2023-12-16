#pragma once

#include <vector>
#include <memory>

class Contact;

class ContactResolver
{
public:
	ContactResolver(int iterations);

	void ResolveContacts(std::vector<std::shared_ptr<Contact>>& contacts, float duration);
	void ResolveVelocity(std::vector<std::shared_ptr<Contact>>& contacts, float duration);
	void ResolveInterpenetration(std::vector<std::shared_ptr<Contact>>& contacts, float duration);

private:
	int iterations;
	int iterationsUsed;
};