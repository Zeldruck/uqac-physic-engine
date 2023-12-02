#pragma once
#include <vector>
#include <memory>

class ParticleContact;

class ParticleContactResolver
{
public:
	ParticleContactResolver(unsigned int iteration);

	void ResolveContacts(std::vector<std::shared_ptr<ParticleContact>>& contactArray, unsigned int numContacts, float duration);

protected:
	unsigned int iteration;
};