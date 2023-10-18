#include "Contact/ParticleContactResolver.hpp"
#include "Contact/ParticleContact.hpp"

ParticleContactResolver::ParticleContactResolver(unsigned int iteration)
{
	this->iteration = iteration;
}

void ParticleContactResolver::ResolveContacts(ParticleContact* contactArray, unsigned int numContact, float duration)
{
	for (int i = 0; i < iteration; i++)
	{
		if (i >= numContact) break;

		(&contactArray[i])->Resolve(duration);
	}
}
