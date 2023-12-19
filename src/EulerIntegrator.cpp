#include <vector>
#include "Quaternion.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"


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

		//rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime /** (1.f - rigidbody->angularDamping)*/;
		//Quaternionf newRotation = Quaternionf(0.f, rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z) * deltaTime;
		//rigidbody->rotation = rigidbody->rotation + newRotation * rigidbody->rotation * 0.5f;
		//rigidbody->rotation.Normalize();

		rigidbody->CalculateDerivedData();
	}
}
