#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleContact.hpp"
#include "PhysicsBody.hpp"

ParticleCable::ParticleCable(std::vector<std::shared_ptr<PhysicsBody>>& particles, float maxLength, float restitution) : 
	ParticleLink(particles)
{
	this->maxLength = maxLength;
	this->restitution = restitution;
}

void ParticleCable::AddContact(std::vector <std::shared_ptr<ParticleContact>>& contact, unsigned int limit)
{
	float penetration = (particles.at(1)->GetPosition() - particles.at(0)->GetPosition()).GetLength() - maxLength;

	if (penetration <= 0) return;

	Vector3f normal = (particles.at(0)->GetPosition() - particles.at(1)->GetPosition()).GetNormalized();

	std::shared_ptr<ParticleContact> newContact = std::make_shared<ParticleContact>(particles, restitution, penetration, normal);

	contact.push_back(newContact);
}
