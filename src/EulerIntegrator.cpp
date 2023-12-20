#include <vector>
#include "Quaternion.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"
#include "Collision/BoundingSphere.hpp"
#include "State.hpp"

void EulerIntegrator::Update(State& current, std::vector<std::shared_ptr<Particle>>& particles, std::vector<std::shared_ptr<Rigidbody>> rigidbodies, const float& deltaTime, bool isGravityEnabled /*= true*/)
{
	// Update Particles
	for (std::shared_ptr<Particle> particle : particles)
	{
		particle->velocity += particle->GetAcceleration() * deltaTime;
		particle->position += particle->velocity * deltaTime;
	}

	// Save Particles Positions
	current.m_particlePositions.clear();
	for (std::shared_ptr<Particle> particle : particles)
	{
		current.m_particlePositions.push_back(particle->position);
	}

	// Update Rigidbodies
	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		rigidbody->velocity += rigidbody->GetAcceleration() * deltaTime * (1.0f - rigidbody->linearDamping);
		rigidbody->position += rigidbody->velocity * deltaTime;
		
		if(rigidbody->m_boundingSphere != nullptr)
			rigidbody->m_boundingSphere->m_center = rigidbody->position;

		rigidbody->angularVelocity += rigidbody->GetAngularAcceleration() * deltaTime * (1.0f - rigidbody->angularDamping);

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

		rigidbody->CalculateDerivedData();
	}

	// Save Rigidbodies Positions & Rotations
	current.m_rigidbodyPositions.clear();
	current.m_rigidbodyRotations.clear();
	for (std::shared_ptr<Rigidbody> rigidbody : rigidbodies)
	{
		current.m_rigidbodyPositions.push_back(rigidbody->position);
		current.m_rigidbodyRotations.push_back(rigidbody->rotation);
	}
}
