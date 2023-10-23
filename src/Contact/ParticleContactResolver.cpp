#include "Contact/ParticleContactResolver.hpp"
#include "Contact/ParticleContact.hpp"

ParticleContactResolver::ParticleContactResolver(unsigned int iteration)
{
	this->iteration = iteration;
}

void ParticleContactResolver::ResolveContacts(std::shared_ptr<std::vector<ParticleContact>> contactArray, unsigned int numContact, float duration)
{
	if(contactArray->size() == 0)
		return;

	for (int i = 0; i < iteration; i++)
	{
		if (i >= contactArray->size()) break;

		contactArray->at(i).Resolve(duration);

		contactArray->erase(contactArray->begin() + i);
	}
}
