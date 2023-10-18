#include "Contact/ParticleRod.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleRod::ParticleRod(Particle* particles[2], float length) : ParticleLink(particles)
{
	this->length = length;
}

unsigned int ParticleRod::AddContact(ParticleContact* contact, unsigned int limit)
{
	// TODO	

	Vector3f d = particles[0]->position - particles[1]->position;

	float penetration = length - d.GetLength();
	Vector3f normal = penetration >= 0.f ? d.GetUnitNormalized() : (particles[1]->position - particles[0]->position).GetUnitNormalized();

	ParticleContact newContact(particles, 0.f, penetration, normal);

	// Add to array

	return 0;
}
