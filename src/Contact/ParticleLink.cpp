#include "Contact/ParticleLink.hpp"
#include "Particle.hpp"

ParticleLink::ParticleLink(std::vector<std::shared_ptr<PhysicsBody>>& particles)
{
    this->particles = particles;
}

float ParticleLink::CurrentLength() const
{
    return (particles.at(1)->GetPosition() - particles.at(1)->GetPosition()).GetLength();
}

void ParticleLink::AddContact(std::vector<std::shared_ptr<ParticleContact>> contact, unsigned int limit)
{
}
