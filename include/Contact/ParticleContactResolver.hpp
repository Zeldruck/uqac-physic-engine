#pragma once
#include <vector>
#include <memory>

class ParticleContact;

class ParticleContactResolver
{
public:
	ParticleContactResolver(unsigned int iteration);

	void ResolveContacts(std::shared_ptr<std::vector<ParticleContact>> contactArray, unsigned int numContact, float duration);

private:

protected:
	unsigned int iteration;
};