#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleCable::ParticleCable(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles, float maxLength, float restitution) : ParticleLink(particles)
{
	this->maxLength = maxLength;
	this->restitution = restitution;
}

void ParticleCable::AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit)
{
	float penetration = (particles->at(1)->position - particles->at(0)->position).GetLength() - maxLength;

	if (penetration <= 0) return;

	Vector3f normal = (particles->at(0)->position - particles->at(1)->position).GetNormalized();

	ParticleContact newContact(particles, restitution, penetration, normal);

	contact->push_back(newContact);
}
