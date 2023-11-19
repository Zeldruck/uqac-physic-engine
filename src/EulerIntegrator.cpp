#include <vector>
#include "Quaternion.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"


void EulerIntegrator::Update(std::vector<std::shared_ptr<Particle>>& particles, std::vector<std::shared_ptr<Rigidbody>> rigidbodies, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	// Update particles postions
	for (std::shared_ptr<Particle> particle : particles)
	{
		Vector3f newPosition = particle->position + particle->velocity * deltaTime;
		particle->position = newPosition;
		particle->velocity += particle->GetAcceleration() * deltaTime;
	}

	// Update Rigidbodies position and rotation
	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		Vector3f newPosition = rigidbody->transform.position + rigidbody->velocity * deltaTime;
		rigidbody->transform.position = newPosition;
		rigidbody->velocity += rigidbody->GetAcceleration() * deltaTime;
		
		rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime;
		Quaternionf newRotation = Quaternionf(0.0f, rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z) * deltaTime;
		newRotation.Normalize();
		rigidbody->transform.rotation = rigidbody->transform.rotation + newRotation * 0.5f;

		rigidbody->CalculateDerivedData();
	}
}