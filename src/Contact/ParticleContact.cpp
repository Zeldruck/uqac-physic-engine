#include "Contact/ParticleContact.hpp"

ParticleContact::ParticleContact(std::vector<std::shared_ptr<Particle>>& particles, float restitution, float penetration, Vector3f contactNormal)
{
	this->particles = particles;

	this->restitution = restitution;
	this->penetration = penetration;
	this->contactNormal = contactNormal;
}

void ParticleContact::Resolve(float duration)
{
	this->duration = duration;

	ResolveVelocity();
	//ResolveInterpenetration();
}

float ParticleContact::CalculateSeparatingVelocity()
{
	return 0.f;
}

void ParticleContact::ResolveVelocity()
{
	float k = Vector3f::DotProduct((restitution + 1) * (particles.at(0)->velocity - particles.at(1)->velocity), contactNormal) /
		(1 / particles.at(0)->mass + 1 / particles.at(1)->mass);
	
	Vector3f v0 = particles.at(0)->velocity - k * duration * contactNormal / particles.at(0)->mass;
	Vector3f v1 = particles.at(1)->velocity + k * duration * contactNormal / particles.at(1)->mass;

	particles.at(0)->velocity = v0;
	particles.at(1)->velocity = v1;
}

void ParticleContact::ResolveInterpenetration()
{
	if (penetration <= 0.f) return;

	Vector3f Pa = (particles.at(1)->mass / particles.at(0)->mass + particles.at(1)->mass) * penetration * contactNormal;
	Vector3f Pb = -(particles.at(0)->mass / particles.at(0)->mass + particles.at(1)->mass) * penetration * contactNormal;

	particles.at(0)->position += Pa;
	particles.at(1)->position += Pb;
}
