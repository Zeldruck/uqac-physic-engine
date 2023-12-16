#include "Collision/ContactResolver.hpp"

ContactResolver::ContactResolver(int iterations)
{
	this->iterations = iterations;
	this->iterationsUsed = 0;
}

void ContactResolver::ResolveContacts(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
	if (contacts.size() == 0) return;

	ResolveVelocity(contacts, duration);
	ResolveInterpenetration(contacts, duration);
}

void ContactResolver::ResolveVelocity(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
}

void ContactResolver::ResolveInterpenetration(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
}
