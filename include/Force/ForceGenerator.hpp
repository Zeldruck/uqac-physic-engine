#pragma once

#include <memory>

class Particle;
class Rigidbody;

class ForceGenerator
{
public:
	virtual void UpdateForce(std::shared_ptr<Particle> physicBody, float deltaTime) = 0;
	virtual void UpdateForce(std::shared_ptr<Rigidbody> physicBody, float deltaTime) = 0;

}; 