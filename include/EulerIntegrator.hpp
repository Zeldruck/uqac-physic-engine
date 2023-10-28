#pragma once

#include <vector>
#include <memory>

#include "PhysicsBody.hpp"
#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"

class EulerIntegrator
{
public:
	void Update(std::vector<std::shared_ptr<PhysicsBody>>& particle, const float& deltaTime, bool isGravityEnabled = true);
private:
	Vector3<float> g = Vector3<float>(0.0f, -GRAVITY, 0.0f);
};