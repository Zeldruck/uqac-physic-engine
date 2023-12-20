#include <vector>
#include "Quaternion.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"
#include "Collision/BoundingSphere.hpp"

#include <iostream>

void EulerIntegrator::Update(std::vector<std::shared_ptr<Particle>>& particles, std::vector<std::shared_ptr<Rigidbody>> rigidbodies, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	// Update Particles
	for (std::shared_ptr<Particle> particle : particles)
	{
		particle->velocity += particle->GetAcceleration() * deltaTime;
		particle->position += particle->velocity * deltaTime;
	}

	// Update Rigidbodies
	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		rigidbody->velocity += rigidbody->GetAcceleration() * deltaTime;
		rigidbody->position += rigidbody->velocity * deltaTime;

		rigidbody->m_boundingSphere->m_center = rigidbody->position;

		rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime;

		// Calculate the rotation quaternion using the angular velocity
		Quaternionf deltaRotation = Quaternionf(
			cos(0.5f * rigidbody->angularVelocity.x * deltaTime),
			sin(0.5f * rigidbody->angularVelocity.x * deltaTime),
			sin(0.5f * rigidbody->angularVelocity.y * deltaTime),
			sin(0.5f * rigidbody->angularVelocity.z * deltaTime)
		);

		// Update the rotation
		rigidbody->rotation = deltaRotation * rigidbody->rotation;
		rigidbody->rotation.Normalize();  // Normalize the quaternion to avoid drift over time

		//Quaternionf newRotation = Quaternionf(0.f, rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z) * deltaTime;
		//newRotation = newRotation * rigidbody->rotation;
		//rigidbody->rotation = rigidbody->rotation + newRotation * 0.5;
		//rigidbody->rotation.Normalize();

		rigidbody->CalculateDerivedData();
	}
}
