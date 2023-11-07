#include <vector>
#include "Quaternion.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"


void EulerIntegrator::Update(std::vector<std::shared_ptr<Particle>>& particles, std::vector<std::shared_ptr<Rigidbody>> rigidbodies, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	for (std::shared_ptr<Particle> particle : particles)
	{
		Vector3f newPosition = particle->position + particle->velocity * deltaTime;
		particle->position = newPosition;
		particle->velocity += particle->GetAcceleration() * deltaTime;
	}

	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		Vector3f newPosition = rigidbody->transform.position + rigidbody->velocity * deltaTime;
		rigidbody->transform.position = newPosition;
		rigidbody->velocity += rigidbody->GetAcceleration() * deltaTime;
		
		Quaternionf current_rotation = rigidbody->transform.rotation;
		Quaternionf q_rate(0, rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
		Quaternionf delta_rotation = /*0.5 * */ deltaTime * q_rate * current_rotation;
		current_rotation += delta_rotation;
		current_rotation.Normalize();
		rigidbody->transform.rotation = current_rotation;
		rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime;
	}
}