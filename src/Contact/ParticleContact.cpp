#include "Contact/ParticleContact.hpp"

ParticleContact::ParticleContact(std::vector<std::shared_ptr<Particle>>& particles, float restitution, float penetration, Vector3f contactNormal)
{
	this->particles = particles;
	this->penetration = penetration;
	this->restitution = restitution;
	this->contactNormal = contactNormal;
}

void ParticleContact::Resolve(float duration)
{
	ResolveVelocity(duration);
	ResolveInterpenetration(duration);
}

float ParticleContact::CalculateSeparatingVelocity()
{
	Vector3f velocity = particles.at(0)->velocity;

	if (particles.at(1))
		velocity -= particles.at(1)->velocity;

	return velocity * contactNormal;
}

void ParticleContact::ResolveVelocity(float duration)
{
	float sVelocity = CalculateSeparatingVelocity();

	if (sVelocity > 0)
		return;

	float newSeparatingVelocity = -sVelocity * restitution;


	Vector3f accel = particles.at(0)->GetAcceleration();
	if (particles.at(1))
		accel -= particles.at(1)->GetAcceleration();

	float accelSeparating = accel * contactNormal * duration;

	if (accelSeparating < 0)
	{
		newSeparatingVelocity += restitution * accelSeparating;
		if (newSeparatingVelocity < 0.f) newSeparatingVelocity = 0.f;
	}


	float inverseMass = 1.f / particles.at(0)->mass;

	if (particles.at(1))
		inverseMass += 1.f / particles.at(1)->mass;

	particles.at(0)->velocity += (contactNormal * (newSeparatingVelocity - sVelocity) / inverseMass) * (1.f / particles.at(0)->mass);
	
	if (particles.at(1))
		particles.at(1)->velocity += (contactNormal * (newSeparatingVelocity - sVelocity) / inverseMass) * -(1.f / particles.at(1)->mass);
}

void ParticleContact::ResolveInterpenetration(float duration)
{
	if (penetration <= 0.f) return;

	float inverseMass = 1.f / particles.at(0)->mass;

	if (particles.at(1))
		inverseMass += 1.f / particles.at(1)->mass;

	particles.at(0)->position += (contactNormal * (-penetration / inverseMass)) * (1.f / particles.at(0)->mass);

	if(particles.at(1))
		particles.at(1)->position += (contactNormal * (-penetration / inverseMass)) * (1.f / particles.at(1)->mass);
}
