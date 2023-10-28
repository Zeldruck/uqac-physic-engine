#pragma once

#include <memory>

class PhysicsBody;

class ForceGenerator
{
public:
	virtual void UpdateForce(std::shared_ptr<PhysicsBody> physicBody, float deltaTime) = 0;
}; 