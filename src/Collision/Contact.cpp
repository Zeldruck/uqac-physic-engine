#include "Collision/Contact.hpp"

Contact::Contact(std::vector<std::shared_ptr<Rigidbody>>& particles, Vector3f contactPoint, Vector3f contactNormal, float penetration, float restitution, Vector3f friction)
{
	this->particles = particles;
	this->contactPoint = contactPoint;
	this->contactNormal = contactNormal;
	this->penetration = penetration;
	this->restitution = restitution;

	this->friction = friction;
}
