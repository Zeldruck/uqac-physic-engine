#pragma once

#include "Vector3.hpp"
#include "Constants/PhysicConstants.hpp"
#include <string>

class Particle
{
public:
	Particle();
	Particle(Vector3f position, Vector3f velocity, Vector3f acceleration, float mass, std::string name = "Particle");
	Particle(const Particle&) = default;
	Particle(Particle&&) = default;
	~Particle() = default;

	Particle& operator=(const Particle&) = default;
	Particle& operator=(Particle&&) = default;

	std::string name;
	Vector3f position;
	Vector3f velocity;
	Vector3f acceleration;
	
	float mass = MIN_MASS;

	void AddForce(const Vector3f& force);
	void RemoveForce(const Vector3f& force);
	void ClearForce();

;	//template <typename T>
	//T& GetComponent() {
	//	// Assuming each component type has a unique string identifier
	//	std::string componentName = typeid(T).name();

	//	// Use dynamic_pointer_cast to safely cast the shared_ptr<Component> to the desired type
	//	return *std::dynamic_pointer_cast<T>(components[componentName]);
	//}

	//template <typename T>
	//void AddComponent(std::shared_ptr<T> component) {
	//	// Assuming each component type has a unique string identifier
	//	std::string componentName = typeid(T).name();

	//	// Use static_pointer_cast to cast the shared_ptr<T> to shared_ptr<Component>
	//	components[componentName] = std::static_pointer_cast<Component>(component);
	//}
};