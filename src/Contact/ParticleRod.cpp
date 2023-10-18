#include "Contact/ParticleRod.hpp"
#include "Particle.hpp"

ParticleRod::ParticleRod(Particle* particles[2], float length) : ParticleLink(particles)
{
	this->length = length;
}

unsigned int ParticleRod::AddContact(ParticleContact* contact, unsigned int limit) const
{
	// TODO

	return 0;
}
