#include "Contact/ParticleRod.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleRod::ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles, float length) : ParticleLink(particles)
{
	this->length = length;
}

void ParticleRod::AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit)
{
	Vector3f d = particles->at(0)->position - particles->at(1)->position;

	float penetration = length - d.GetLength();

	if (penetration < 0.01f && penetration > 0.01f) return;

	Vector3f normal = penetration >= 0.f ? d.GetUnitNormalized() : (particles->at(1)->position - particles->at(0)->position).GetUnitNormalized();

	ParticleContact newContact(particles, 0.f, penetration, normal);

	// Add to array
	contact->push_back(newContact);
}
