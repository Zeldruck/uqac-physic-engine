#include "Contact/ParticleLink.hpp"
#include "Particle.hpp"

ParticleLink::ParticleLink(std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles)
{
    this->particles = particles;
}

float ParticleLink::CurrentLength() const
{
    return (particles->at(1)->position - particles->at(1)->position).GetLength();
}

void ParticleLink::AddContact(std::shared_ptr<std::vector<ParticleContact>> contact, unsigned int limit)
{
}
