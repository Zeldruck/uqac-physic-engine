#include "Collision/ContactGenerator.hpp"
#include "Collision/Contact.hpp"
#include "Collision/Primitives/Primitive.hpp"
#include "Collision/Primitives/Sphere.hpp"
#include "Rigidbody.hpp"

ContactGenerator::ContactGenerator(float maxContacts)
{
	this->maxContacts = maxContacts;
	currentContacts = 0;
}

void ContactGenerator::DetectContacts(const Primitive& primitiveA, const Primitive& primitiveB)
{
}

void ContactGenerator::DetectSandS(const Sphere& sphereA, const Sphere& sphereB)
{
	if (currentContacts >= maxContacts) return;

	Vector3f posA = sphereA.rigidbody->transform.position;
	Vector3f posB = sphereB.rigidbody->transform.position;

	float distance = (posA - posB).GetLength();

	if (distance <= 0.f || distance >= sphereA.radius + sphereB.radius) return;

	std::shared_ptr<Contact> contact = std::make_shared<Contact>();
	contact->contactNormal = (posA - posB) * (1.f / distance);
	contact->contactPoint = posA + (posA - posB) * 0.5f;
	contact->penetration = sphereA.radius + sphereB.radius - distance;

	//contact->restitution = ;
	//contact->friction = ;

	std::vector<std::shared_ptr<Rigidbody>> rbs;
	rbs.push_back(sphereA.rigidbody);
	rbs.push_back(sphereB.rigidbody);

	contact->particles = rbs;
}
