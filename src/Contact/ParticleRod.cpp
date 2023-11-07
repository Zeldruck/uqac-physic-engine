#include "Contact/ParticleRod.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleRod::ParticleRod(std::vector<std::shared_ptr<Particle>>& particles, float length) :
	ParticleLink(particles)
{
	this->length = length;
}

void ParticleRod::AddContact(std::vector<std::shared_ptr<ParticleContact>>& contact, unsigned int limit)
{
	Vector3f d = particles.at(1)->position - particles.at(0)->position;

	float penetration = length - d.GetLength();

	if (penetration < 0.05f && penetration > -0.05f) return;


	Vector3f normal = (particles.at(0)->position - particles.at(1)->position).GetNormalized();

	std::shared_ptr<ParticleContact> newContact = std::make_shared<ParticleContact>(particles, 0.5f, penetration, normal);

	contact.push_back(newContact);
}
