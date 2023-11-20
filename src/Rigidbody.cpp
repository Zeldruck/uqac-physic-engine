#include "Rigidbody.hpp"
#include "Constants/PhysicConstants.hpp"
#include "Constants/MathConstants.hpp"
#include "Matrix4.hpp"
#include "Matrix3.hpp"
#include "Quaternion.hpp"
#include "Particle.hpp"

Rigidbody::Rigidbody()
	:
	transform(Transform()),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	inertiaTensor(GetBoxInertiaTensorLocal()),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	type(RigidbodyType::BOX),
	massPoints(std::vector<Particle>())
{
	CalculateDerivedData();
	CalculateCenterOfMass();
}

Rigidbody::Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name, RigidbodyType type, std::vector<Particle> massPoints /*= std::vector<Particle>()*/, float linearDamping/* = 0.0f*/, float angularDamping/*= 0.5f*/)
	:
	transform(transform),
	velocity(velocity),
	m_acceleration(acceleration),
	force(Vector3f::Zero),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	angularVelocity(angularVelocity),
	m_angularAcceleration(angularAcceleration),
	inertiaTensor(GetBoxInertiaTensorLocal()),
	name(name),
	type(type),
	massPoints(std::vector<Particle>()),
	linearDamping(linearDamping),
	angularDamping(angularDamping)
{
	CalculateDerivedData();
	CalculateCenterOfMass();
	inverseMass = 1.0f / mass;
}

void Rigidbody::ClearForce()
{
	force = Vector3f::Zero;
}

void Rigidbody::ClearTorque()
{
	torque = Vector3f::Zero;
}

void Rigidbody::AddForce(const Vector3f& f)
{
	force += f;
}

void Rigidbody::RemoveForce(const Vector3f& f)
{
	force -= f;
}

Vector3f Rigidbody::GetPointInWorldSpace(const Vector3f& point)
{
	return transformMatrix * point;
}

void Rigidbody::AddForceAtPoint(const Vector3f& f, const Vector3f& point)
{
	Vector3f pt = point;
	pt -= transform.position;
	force += f;
	torque += pt.Cross(f);
}

void Rigidbody::AddForceAtBodyPoint(const Vector3f& f, const Vector3f& point)
{
	Vector3f pt = GetPointInWorldSpace(point);
	AddForceAtPoint(f, pt);
}

void Rigidbody::RemoveForceAtPoint(const Vector3f& f, const Vector3f& point)
{
	force -= f;
	torque -= (point - transform.position).Cross(f);
}

void Rigidbody::RemoveForceAtBodyPoint(const Vector3f& f, const Vector3f& point)
{
	force -= f;
	torque -= point.Cross(f);
}

Vector3f const Rigidbody::GetAcceleration()
{
	m_acceleration = force * inverseMass;
	return m_acceleration;
}

void Rigidbody::SetAcceleration(const Vector3f& acceleration)
{
	m_acceleration = acceleration;
}

Vector3f const Rigidbody::GetAngularAcceleration()
{
	m_angularAcceleration = inverseInertiaTensor * torque;
	return m_angularAcceleration;
}

void Rigidbody::SetAngularAcceleration(const Vector3f& acceleration)
{
	m_angularAcceleration = acceleration;
}

void Rigidbody::CalculateTransformMatrix()
{
	float x = transform.rotation.GetX();
	float y = transform.rotation.GetY();
	float z = transform.rotation.GetZ();
	float s = transform.rotation.GetS();
	float posX = transform.position.x;
	float posY = transform.position.y;
	float posZ = transform.position.z;

	transformMatrix = Matrix4f({
		1.0f - 2.0f * y * y - 2.0f * z * z,     2.0f * x * y - 2.0f * s * z,			2.0f * x * z + 2.0f * s * y,			posX,
		2.0f * x * y + 2.0f * s * z,			1.0f - 2.0f * x * x - 2.0f * z * z,		2.0f * y * z - 2.0f * s * x,			posY,
		2.0f * x * z - 2.0f * s * y,			2.0f * y * z + 2.0f * s * x,			1.0f - 2.0f * x * x - 2.0f * y * y,		posZ,
		0.0f,									0.0f,									0.0f,									1.0f
		}
	);
}

void Rigidbody::CalculateDerivedData()
{
	CalculateTransformMatrix();
	CalculateInverseInertiaTensorWorld();
}

Matrix3f Rigidbody::GetBoxInertiaTensorLocal()
{
	float mass = this->mass;
	Vector3f scale = transform.scale;

	float Ixx = (mass / 12.0f) * (scale.y * scale.y + scale.z * scale.z);
	float Iyy = (mass / 12.0f) * (scale.x * scale.x + scale.z * scale.z);
	float Izz = (mass / 12.0f) * (scale.x * scale.x + scale.y * scale.y);

	return Matrix3f({
		Ixx, 0.0f, 0.0f,
		0.0f, Iyy, 0.0f,
		0.0f, 0.0f, Izz
		});
}

Matrix3f Rigidbody::GetBoxInertiaTensorWorld()
{
	inertiaTensor = GetBoxInertiaTensorLocal();
	CalculateInverseInertiaTensor();
	return GetInertiaTensorWorld();
}


Matrix3f Rigidbody::GetSphereInertiaTensorLocal()
{
	float radius = transform.scale.x;

	float I = (2.0f / 5.0f) * (mass * radius * radius);

	return Matrix3f({
		I, 0.0f, 0.0f,
		0.0f, I, 0.0f,
		0.0f, 0.0f, I
		});
}

Matrix3f Rigidbody::GetSphereInertiaTensorWorld()
{
	inertiaTensor = GetSphereInertiaTensorLocal();
	CalculateInverseInertiaTensor();
	return GetInertiaTensorWorld();
}

void Rigidbody::SetInertiaTensor(const Matrix3f& _inertiaTensor)
{
	inertiaTensor = _inertiaTensor;
}

void Rigidbody::CalculateInverseInertiaTensor()
{
	inverseInertiaTensor = inertiaTensor.Inverse();
}

void Rigidbody::CalculateInverseInertiaTensorWorld()
{
	switch (type) 
	{
	case RigidbodyType::BOX:
		inverseInertiaTensor = GetBoxInertiaTensorWorld().Inverse();
		break;
	case RigidbodyType::SPHERE:
		inverseInertiaTensor = GetSphereInertiaTensorWorld().Inverse();
		break;
	case RigidbodyType::TRIANGLE:
		inverseInertiaTensor = GetTriangleInertiaTensorWorld().Inverse();
		break;
	default:
		inverseInertiaTensor = GetBoxInertiaTensorWorld().Inverse();
	}
}

Matrix3f Rigidbody::GetTriangleInertiaTensorLocal()
{
	// Calculate the center of mass
	Vector3f v0 = Vector3f::One;
	Vector3f v1 = Vector3f::One;
	Vector3f v2 = Vector3f::One;

	Vector3f centerOfMass = (v0 + v1 + v2) / 3.0f;

	// Calculate the mass of the triangle
	float area = 0.5f * Vector3f::CrossProduct(v1 - v0, v2 - v0).GetLength();  // Area of the triangle

	// Calculate the moment arm for each vertex
	Vector3f r0 = v0 - centerOfMass;
	Vector3f r1 = v1 - centerOfMass;
	Vector3f r2 = v2 - centerOfMass;

	// Calculate the moment of inertia components
	float Ixx = mass * (r0.y * r0.y + r0.z * r0.z + r1.y * r1.y + r1.z * r1.z + r2.y * r2.y + r2.z * r2.z);
	float Iyy = mass * (r0.x * r0.x + r0.z * r0.z + r1.x * r1.x + r1.z * r1.z + r2.x * r2.x + r2.z * r2.z);
	float Izz = mass * (r0.x * r0.x + r0.y * r0.y + r1.x * r1.x + r1.y * r1.y + r2.x * r2.x + r2.y * r2.y);
	float Ixy = -mass * (r0.x * r0.y + r1.x * r1.y + r2.x * r2.y);
	float Ixz = -mass * (r0.x * r0.z + r1.x * r1.z + r2.x * r2.z);
	float Iyz = -mass * (r0.y * r0.z + r1.y * r1.z + r2.y * r2.z);

	// Populate the inertia tensor matrix
	inertiaTensor = Matrix3f(
		{
			Ixx, Ixy, Ixz, 
			Ixy, Iyy, Iyz, 
			Ixz, Iyz, Izz
		});

	return inertiaTensor;
}

Matrix3f Rigidbody::GetTriangleInertiaTensorWorld()
{
	inertiaTensor = GetTriangleInertiaTensorLocal();
	CalculateInverseInertiaTensor();
	return GetInertiaTensorWorld();
}

Matrix3f Rigidbody::GetInertiaTensorWorld()
{
	Matrix3f iitLocal = inverseInertiaTensor;
	Quaternion q = transform.rotation;
	Matrix4f rotM = transformMatrix;

	float t4 = rotM.Value(0, 0) * iitLocal.Value(0, 0) +
		rotM.Value(0, 1) * iitLocal.Value(1, 0) +
		rotM.Value(0, 2) * iitLocal.Value(2, 0);
	float t9 = rotM.Value(0, 0) * iitLocal.Value(0, 1) +
		rotM.Value(0, 1) * iitLocal.Value(1, 1) +
		rotM.Value(0, 2) * iitLocal.Value(2, 1);
	float t14 = rotM.Value(0, 0) * iitLocal.Value(0, 2) +
		rotM.Value(0, 1) * iitLocal.Value(1, 2) +
		rotM.Value(0, 2) * iitLocal.Value(2, 2);
	float t28 = rotM.Value(1, 0) * iitLocal.Value(0, 0) +
		rotM.Value(1, 1) * iitLocal.Value(1, 0) +
		rotM.Value(1, 2) * iitLocal.Value(2, 0);
	float t33 = rotM.Value(1, 0) * iitLocal.Value(0, 1) +
		rotM.Value(1, 1) * iitLocal.Value(1, 1) +
		rotM.Value(1, 2) * iitLocal.Value(2, 1);
	float t38 = rotM.Value(1, 0) * iitLocal.Value(0, 2) +
		rotM.Value(1, 1) * iitLocal.Value(1, 2) +
		rotM.Value(1, 2) * iitLocal.Value(2, 2);
	float t52 = rotM.Value(2, 0) * iitLocal.Value(0, 0) +
		rotM.Value(2, 1) * iitLocal.Value(1, 0) +
		rotM.Value(2, 2) * iitLocal.Value(2, 0);
	float t57 = rotM.Value(2, 0) * iitLocal.Value(0, 1) +
		rotM.Value(2, 1) * iitLocal.Value(1, 1) +
		rotM.Value(2, 2) * iitLocal.Value(2, 1);
	float t62 = rotM.Value(2, 0) * iitLocal.Value(0, 2) +
		rotM.Value(2, 1) * iitLocal.Value(1, 2) +
		rotM.Value(2, 2) * iitLocal.Value(2, 2);

	Matrix3f iitWorld = Matrix3f({
		t4 * rotM.Value(0, 0) + t9 * rotM.Value(0, 1) + t14 * rotM.Value(0, 2),
		t4 * rotM.Value(1, 0) + t9 * rotM.Value(1, 1) + t14 * rotM.Value(1, 2),
		t4 * rotM.Value(2, 0) + t9 * rotM.Value(2, 1) + t14 * rotM.Value(2, 2),
		t28 * rotM.Value(0, 0) + t33 * rotM.Value(0, 1) + t38 * rotM.Value(0, 2),
		t28 * rotM.Value(1, 0) + t33 * rotM.Value(1, 1) + t38 * rotM.Value(1, 2),
		t28 * rotM.Value(2, 0) + t33 * rotM.Value(2, 1) + t38 * rotM.Value(2, 2),
		t52 * rotM.Value(0, 0) + t57 * rotM.Value(0, 1) + t62 * rotM.Value(0, 2),
		t52 * rotM.Value(1, 0) + t57 * rotM.Value(1, 1) + t62 * rotM.Value(1, 2),
		t52 * rotM.Value(2, 0) + t57 * rotM.Value(2, 1) + t62 * rotM.Value(2, 2)
		});

	return iitWorld;
}

void Rigidbody::CalculateInertiaMatrix()
{
	float Ixx = 0.0f;
	float Iyy = 0.0f;
	float Izz = 0.0f;
	float Ixy = 0.0f;
	float Ixz = 0.0f;
	float Iyz = 0.0f;

	//// Calculate the product of inertia components
	//for (const Particle& massPoint : massPoints) {
	//	double x = massPoint.position.x;
	//	double y = massPoint.position.y;
	//	double z = massPoint.position.z;
	//	double m = massPoint.mass;

	//	// Update the matrix components
	//	inertiaMatrix[0][0] += m * (y * y + z * z);
	//	inertiaMatrix[0][1] -= m * (x * y);
	//	inertiaMatrix[0][2] -= m * (x * z);
	//	inertiaMatrix[1][0] -= m * (x * y);
	//	inertiaMatrix[1][1] += m * (x * x + z * z);
	//	inertiaMatrix[1][2] -= m * (y * z);
	//	inertiaMatrix[2][0] -= m * (x * z);
	//	inertiaMatrix[2][1] -= m * (y * z);
	//	inertiaMatrix[2][2] += m * (x * x + y * y);
	//}

	//return inertiaMatrix;
}

void Rigidbody::CalculateCenterOfMass()
{
	float totalMass = 0.0f;
	Vector3f totalPosition = Vector3f::Zero;

	for (const Particle& massPoint : massPoints) 
	{
		totalMass += massPoint.mass;
		totalPosition += massPoint.position;
	}

	if (totalMass != 0.0f) 
	{
		centerOfMass = (totalMass * totalPosition) / mass;
	}
	else
	{
		centerOfMass = Vector3f::Zero;
	}
}
