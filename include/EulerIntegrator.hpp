#pragma once

#include <vector>
#include <memory>

#include "Particle.hpp"
#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"

class Particle;

class EulerIntegrator
{
public:
	void Update(std::vector<std::shared_ptr<Particle>>& particle, const float& deltaTime, bool isGravityEnabled = true);
private:
	Vector3<float> g = Vector3<float>(0.0f, -GRAVITY, 0.0f);
};