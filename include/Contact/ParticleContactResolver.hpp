#pragma once

class ParticleContact;

class ParticleContactResolver
{
public:
	ParticleContactResolver(unsigned int iteration);
	~ParticleContactResolver();

	void ResolveContacts(ParticleContact* conactArray, unsigned int numContact, float duration);

private:

protected:
	unsigned int iteration;
};