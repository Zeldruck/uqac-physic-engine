#include "Contact/ParticleContactResolver.hpp"
#include "Contact/ParticleContact.hpp"

ParticleContactResolver::ParticleContactResolver(unsigned int iteration)
{
	this->iteration = iteration;
}

void ParticleContactResolver::ResolveContacts(std::vector<std::shared_ptr<ParticleContact>>& contactArray, unsigned int numContacts, float duration)
{
	if(contactArray.size() == 0)
		return;

	unsigned int iterationused = 0;

	while (iterationused < iteration)
	{
		float max = 0.f;
		int maxIndex = numContacts;

		for (int i = 0; i < numContacts; i++)
		{
			float separatingVelocity = contactArray.at(i)->CalculateSeparatingVelocity();

			if (separatingVelocity < max)
			{
				max = separatingVelocity;
				maxIndex = i;
			}
		}

		contactArray.at(maxIndex)->Resolve(duration);
		iterationused++;
	}

	for (int i = 0; i < iteration; i++)
	{
		if (i >= contactArray.size()) break;

		contactArray.at(i)->Resolve(duration);

		contactArray.erase(contactArray.begin() + i);
	}
}
