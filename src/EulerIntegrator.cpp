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
		particle->velocity += particle->GetAcceleration() * deltaTime;
		particle->position += particle->velocity * deltaTime;
	}

	// Update Rigidbodies position and rotation
	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		Vector3f newPosition = rigidbody->transform.position + rigidbody->velocity * deltaTime * (1.f - rigidbody->linearDamping);
		rigidbody->transform.position = newPosition;
		rigidbody->velocity += rigidbody->GetAcceleration() * deltaTime;
		

		// For test purpose only
		//rigidbody->angularVelocity = Vector3f(10.0f, 0.0f, 10.f);
		//----- 
		
		rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime * (1.f - rigidbody->angularDamping);
		Quaternionf newRotation = Quaternionf(0.f, rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z) * deltaTime;
		rigidbody->transform.rotation = rigidbody->transform.rotation + newRotation * rigidbody->transform.rotation * 0.5f;
		rigidbody->transform.rotation.Normalize();

		rigidbody->CalculateDerivedData();
	}
}
