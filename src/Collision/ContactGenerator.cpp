#include "Collision/ContactGenerator.hpp"
#include "Collision/Contact.hpp"
#include "Collision/Primitives/Primitive.hpp"
#include "Collision/Primitives/Sphere.hpp"
#include "Collision/Primitives/Plane.hpp"
#include "Collision/Primitives/Box.hpp"
#include "Rigidbody.hpp"

#include <math.h>

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

	std::vector<std::shared_ptr<Rigidbody>> rbs;
	rbs.push_back(sphereA.rigidbody);
	rbs.push_back(sphereB.rigidbody);

	contact->particles = rbs;

	contacts.push_back(contact);

	currentContacts++;
}

void ContactGenerator::DetectSandHS(const Sphere& sphere, const Plane& plane)
{
	if (currentContacts >= maxContacts) return;

	Vector3f sPos = sphere.rigidbody->transform.position;

	float distanceFromPlane = plane.normal * sPos - sphere.radius - plane.offset;

	if (distanceFromPlane >= 0) return; // No collision

	std::shared_ptr<Contact> contact = std::make_shared<Contact>();
	contact->contactNormal = plane.normal;
	contact->contactPoint = sPos - plane.normal * (distanceFromPlane + sphere.radius);
	contact->penetration = -distanceFromPlane;

	std::vector<std::shared_ptr<Rigidbody>> rbs;
	rbs.push_back(sphere.rigidbody);
	rbs.push_back(plane.rigidbody);

	contact->particles = rbs;

	contacts.push_back(contact);

	currentContacts++;
}

void ContactGenerator::DetectSandP(const Sphere& sphere, const Plane& plane)
{
	if (currentContacts >= maxContacts) return;

	Vector3f sPos = sphere.rigidbody->transform.position;

	float distance = plane.normal * sPos - plane.offset;

	if (distance * distance > sphere.radius * sphere.radius) return;

	std::shared_ptr<Contact> contact = std::make_shared<Contact>();
	contact->contactNormal = distance < 0 ? plane.normal*-1.f : plane.normal;
	contact->contactPoint = sPos - plane.normal * distance;
	contact->penetration = distance < 0 ? distance : -distance;

	std::vector<std::shared_ptr<Rigidbody>> rbs;
	rbs.push_back(sphere.rigidbody);
	rbs.push_back(plane.rigidbody);

	contact->particles = rbs;

	contacts.push_back(contact);

	currentContacts++;
}

void ContactGenerator::DetectSandB(const Sphere& sphere, const Box& box)
{
	if (currentContacts >= maxContacts) return;

	Vector3f center = sphere.rigidbody->transform.position;
	Vector3f rCenter = box.rigidbody->transformMatrix.TransformInverse(center);
	Vector3f closestPoint;
	float distance = center.x;

	if (distance > box.halfSize.x) distance = box.halfSize.x;
	if (distance < -box.halfSize.x) distance = -box.halfSize.x;
	closestPoint.x = distance;

	distance = center.y;
	if (distance > box.halfSize.y) distance = box.halfSize.y;
	if (distance < -box.halfSize.y) distance = -box.halfSize.y;
	closestPoint.y = distance;

	if (distance > box.halfSize.z) distance = box.halfSize.z;
	if (distance < -box.halfSize.z) distance = -box.halfSize.z;
	closestPoint.y = distance;

	distance = (closestPoint - rCenter).GetLengthSquared();

	if (distance > sphere.radius * sphere.radius) return;

	Vector3f closestPointWorld = box.rigidbody->transformMatrix * closestPoint;

	std::shared_ptr<Contact> contact = std::make_shared<Contact>();
	contact->contactNormal = (closestPointWorld - center).GetNormalized();
	contact->contactPoint = closestPointWorld;
	contact->penetration = sphere.radius - std::sqrt(distance);

	std::vector<std::shared_ptr<Rigidbody>> rbs;
	rbs.push_back(box.rigidbody);
	rbs.push_back(sphere.rigidbody);

	contact->particles = rbs;

	contacts.push_back(contact);

	currentContacts++;
}

void ContactGenerator::DetectBandP(const Box& box, const Plane& plane)
{
	if (currentContacts >= maxContacts) return;

	Vector3f vertices[8] = 
	{
		Vector3f(-box.halfSize.x, -box.halfSize.y, -box.halfSize.z),
		Vector3f(-box.halfSize.x, -box.halfSize.y, box.halfSize.z),
		Vector3f(-box.halfSize.x, box.halfSize.y, -box.halfSize.z),
		Vector3f(-box.halfSize.x, box.halfSize.y, box.halfSize.z),
		Vector3f(box.halfSize.x, -box.halfSize.y, -box.halfSize.z),
		Vector3f(box.halfSize.x, -box.halfSize.y, box.halfSize.z),
		Vector3f(box.halfSize.x, box.halfSize.y, -box.halfSize.z),
		Vector3f(box.halfSize.x, box.halfSize.y, box.halfSize.z)
	};

	for (auto i = 0; i < 8; i++)
	{
		vertices[i] = box.offset * vertices[i];

		float distance = vertices[i] * plane.normal;

		if (distance > plane.offset) continue;

		std::shared_ptr<Contact> contact = std::make_shared<Contact>();
		contact->contactNormal = plane.normal;
		contact->contactPoint = plane.normal * (distance - plane.offset) + vertices[i];
		contact->penetration = plane.offset - distance;

		std::vector<std::shared_ptr<Rigidbody>> rbs;
		rbs.push_back(plane.rigidbody);
		rbs.push_back(nullptr);

		contact->particles = rbs;

		contacts.push_back(contact);

		currentContacts++;

		if (currentContacts >= maxContacts) break;
	}
}
