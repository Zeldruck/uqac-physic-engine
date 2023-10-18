#include "Contact/ParticleLink.hpp"
#include "Particle.hpp"

ParticleLink::ParticleLink(Particle* particles[2])
{
    this->particles[0] = particles[0];
    this->particles[1] = particles[1];
}

float ParticleLink::CurrentLength() const
{
    return (particles[0]->position - particles[1]->position).GetLength();
}
