#pragma once

#include <vector>
#include <memory>

#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"

class Particle;
class Rigidbody;
struct State;

class EulerIntegrator
{
public:
	EulerIntegrator() = default;

	void Update(State& current, std::vector<std::shared_ptr<Particle>>& particles, std::vector<std::shared_ptr<Rigidbody>> rigidbodies, const float& deltaTime, bool isGravityEnabled = true);
private:
	Vector3<float> g = Vector3<float>(0.0f, -GRAVITY, 0.0f);
};