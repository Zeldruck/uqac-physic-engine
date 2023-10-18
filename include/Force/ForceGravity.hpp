#pragma once

#include "Vector3.hpp"
#include "ForceGenerator.hpp"
#include "Constants/PhysicConstants.hpp"
class ForceGravity : public ForceGenerator
{
private :
	Vector3f m_gravity = Vector3f(0.f, -GRAVITY, 0.f);

public :
	void UpdateForce(std::shared_ptr<Particle> particle, float deltaTime) override;
};