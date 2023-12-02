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
	float currLength = CurrentLength();

	if (currLength == length) return;

	Vector3f normal = (particles.at(1)->position - particles.at(0)->position).GetNormalized();
	float penetration = 0.f;

	if (currLength > length)
	{
		penetration = currLength - length;
	}
	else
	{
		normal = normal * -1.f;
		penetration = length - currLength;
	}

	std::shared_ptr<ParticleContact> newContact = std::make_shared<ParticleContact>(particles, 0, penetration, normal);

	contact.push_back(newContact);
}
