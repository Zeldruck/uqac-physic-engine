#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleContact.hpp"
#include "Particle.hpp"

ParticleCable::ParticleCable(Particle* particles[2], float maxLength, float restitution) : ParticleLink(particles)
{
	this->maxLength = maxLength;
	this->restitution = restitution;
}

unsigned int ParticleCable::AddContact(ParticleContact* contact, unsigned int limit)
{
	// TODO

	float penetration = maxLength - (particles[1]->position - particles[0]->position).GetLength();
	Vector3f normal = (particles[1]->position - particles[0]->position).GetUnitNormalized();

	ParticleContact newContact(particles, restitution, penetration, normal);

	// Add to array

	return 0;
}
