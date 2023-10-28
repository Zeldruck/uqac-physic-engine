#include "Contact/ParticleRod.hpp"
#include "Contact/ParticleContact.hpp"
#include "PhysicsBody.hpp"

ParticleRod::ParticleRod(std::vector<std::shared_ptr<PhysicsBody>>& particles, float length) :
	ParticleLink(particles)
{
	this->length = length;
}

void ParticleRod::AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit)
{
	Vector3f d = particles.at(1)->GetPosition() - particles.at(0)->GetPosition();

	float penetration = length - d.GetLength();

	if (penetration < 0.05f && penetration > -0.05f) return;

	Vector3f normal = (particles.at(0)->GetPosition() - particles.at(1)->GetPosition()).GetNormalized();

	std::shared_ptr<ParticleContact> newContact = std::make_shared<ParticleContact>(particles, 0.5f, penetration, normal);

	contact.push_back(newContact);
}
