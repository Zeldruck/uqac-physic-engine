#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleCable::ParticleCable(std::vector<std::shared_ptr<Particle>>& particles, float maxLength, float restitution) : 
	ParticleLink(particles)
{
	this->maxLength = maxLength;
	this->restitution = restitution;
}

void ParticleCable::AddContact(std::vector <std::shared_ptr<ParticleContact>>& contact, unsigned int limit)
{
	float length = CurrentLength();

	if (length < maxLength) return;

	Vector3f normal = (particles.at(1)->position - particles.at(0)->position).GetNormalized();

	std::shared_ptr<ParticleContact> newContact = std::make_shared<ParticleContact>(particles, restitution, length - maxLength, normal);

	contact.push_back(newContact);
}
