#include "Contact/ParticleCable.hpp"
#include "Particle.hpp"

ParticleCable::ParticleCable(Particle* particles[2], float maxLength, float restitution) : ParticleLink(particles)
{
	this->maxLength = maxLength;
	this->restitution = restitution;
}

unsigned int ParticleCable::AddContact(ParticleContact* contact, unsigned int limit) const
{
	// TODO

	return 0;
}
