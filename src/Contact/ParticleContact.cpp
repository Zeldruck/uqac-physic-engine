#include "Contact/ParticleContact.hpp"

ParticleContact::ParticleContact(Particle* particles, float restitution)
{
	this->particles[0] = &particles[0];
	this->particles[1] = &particles[1];
	this->restitution = restitution;
}

void ParticleContact::Resolve(float duration)
{
	ResolveVelocity();
	ResolveInterpenetration();
}

float ParticleContact::CalculateSeparatingVelocity()
{
	return 0.f;
}

void ParticleContact::ResolveVelocity()
{
	float k = Vector3f::DotProduct((restitution + 1) * (particles[0]->velocity - particles[1]->velocity), contactNormal) /
		(1 / particles[0]->mass + 1 / particles[1]->mass);
	
	Vector3f v0 = particles[0]->velocity - k * contactNormal / particles[0]->mass;
	Vector3f v1 = particles[1]->velocity + k * contactNormal / particles[1]->mass;

	// TODO
}

void ParticleContact::ResolveInterpenetration()
{
	if (penetration <= 0.f) return;

	Vector3f Pa = (particles[1]->mass / particles[0]->mass + particles[1]->mass) * penetration * contactNormal;
	Vector3f Pb = -(particles[0]->mass / particles[0]->mass + particles[1]->mass) * penetration * contactNormal;

	// TODO

	// Gestion contact au repos
}
