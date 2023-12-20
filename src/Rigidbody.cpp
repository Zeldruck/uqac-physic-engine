#include "Rigidbody.hpp"
#include "Constants/PhysicConstants.hpp"
#include "Constants/MathConstants.hpp"
#include "Collision/BoundingSphere.hpp"
#include "Collision/BoundingBox.hpp"

Rigidbody::Rigidbody() :
	name("Rigidbody"),
	type(SPHERE),
	position(Vector3f::Zero),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	inertiaTensor(GetSphereInertiaTensorLocal()),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name) :
	name(name),
	type(SPHERE),
	position(Vector3f::Zero),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	inertiaTensor(GetSphereInertiaTensorLocal()),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, Vector3f position) :
	name(name),
	type(SPHERE),
	position(position),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	inertiaTensor(GetSphereInertiaTensorLocal()),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, Vector3f position, float mass) :
	name(name),
	type(SPHERE),
	position(position),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	inertiaTensor(GetSphereInertiaTensorLocal()),
	mass(mass),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, RigidbodyType type) :
	name(name),
	type(type),
	position(Vector3f::Zero),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	switch (type)
	{
	case CUBE:
		inertiaTensor = GetBoxInertiaTensorLocal();
		break;
	case SPHERE:
		inertiaTensor = GetSphereInertiaTensorLocal();
		break;
	case TETRAHEDRON:
		inertiaTensor = GetTetrahedronInertiaTensorLocal();
		break;
	}

	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, RigidbodyType type, Vector3f position) :
	name(name),
	type(type),
	position(position),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	mass(MIN_MASS),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	switch (type)
	{
	case CUBE:
		inertiaTensor = GetBoxInertiaTensorLocal();
		break;
	case SPHERE:
		inertiaTensor = GetSphereInertiaTensorLocal();
		break;
	case TETRAHEDRON:
		inertiaTensor = GetTetrahedronInertiaTensorLocal();
		break;
	}

	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, RigidbodyType type, Vector3f position, float mass) :
	name(name),
	type(type),
	position(position),
	rotation(Quaternionf()),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	mass(mass),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	switch (type)
	{
	case CUBE:
		inertiaTensor = GetBoxInertiaTensorLocal();
		break;
	case SPHERE:
		inertiaTensor = GetSphereInertiaTensorLocal();
		break;
	case TETRAHEDRON:
		inertiaTensor = GetTetrahedronInertiaTensorLocal();
		break;
	}

	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, RigidbodyType type, Vector3f position, Quaternionf rotation, float mass) :
	name(name),
	type(type),
	position(position),
	rotation(rotation),
	scale(Vector3f::One),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	mass(mass),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	switch (type)
	{
	case CUBE:
		inertiaTensor = GetBoxInertiaTensorLocal();
		break;
	case SPHERE:
		inertiaTensor = GetSphereInertiaTensorLocal();
		break;
	case TETRAHEDRON:
		inertiaTensor = GetTetrahedronInertiaTensorLocal();
		break;
	}

	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
}

Rigidbody::Rigidbody(std::string name, RigidbodyType, Vector3f position, Quaternionf rotation, Vector3f scale, float mass) :
	name(name),
	type(type),
	position(position),
	rotation(rotation),
	scale(scale),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	mass(mass),
	inverseMass(1.0f / MIN_MASS),
	isAwake(true)
{
	switch (type)
	{
	case CUBE:
		inertiaTensor = GetBoxInertiaTensorLocal();
		break;
	case SPHERE:
		inertiaTensor = GetSphereInertiaTensorLocal();
		break;
	case TETRAHEDRON:
		inertiaTensor = GetTetrahedronInertiaTensorLocal();
		break;
	}

	inverseInertiaTensor = inertiaTensor.Inverse();
	CalculateDerivedData();
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

Vector3f Rigidbody::GetPointInWorldSpace(const Vector3f& point)
{
	return transformMatrix * point;
}

Vector3f Rigidbody::GetPointInLocalSpace(const Vector3f& point)
{
	return transformMatrix.Inverse() * point;
}

void Rigidbody::AddForceAtPoint(const Vector3f& f, const Vector3f& point)
{
	Vector3f pt = point;
	pt -= position;
	force += f;
	torque += pt.Cross(f);
}

void Rigidbody::AddForceAtBodyPoint(const Vector3f& f, const Vector3f& point)
{
	Vector3f pt = GetPointInWorldSpace(point);
	AddForceAtPoint(f, pt);
}

Vector3f const Rigidbody::GetAcceleration()
{
	m_acceleration = force / mass;
	return m_acceleration;
}

Vector3f const Rigidbody::GetAngularAcceleration()
{
	m_angularAcceleration = inverseInertiaTensorWorld * torque;
	return m_angularAcceleration;
}

void Rigidbody::CalculateTransformMatrix()
{
	float x = rotation.GetX();
	float y = rotation.GetY();
	float z = rotation.GetZ();
	float s = rotation.GetS();
	float posX = position.x;
	float posY = position.y;
	float posZ = position.z;

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
	inverseInertiaTensorWorld = GetInverseInertiaTensorWorld();
}

Matrix3f Rigidbody::GetBoxInertiaTensorLocal()
{
	float mass = this->mass;

	float Ixx = (mass / 12.0f) * (scale.y * scale.y + scale.z * scale.z);
	float Iyy = (mass / 12.0f) * (scale.x * scale.x + scale.z * scale.z);
	float Izz = (mass / 12.0f) * (scale.x * scale.x + scale.y * scale.y);

	return Matrix3f({
		Ixx, 0.0f, 0.0f,
		0.0f, Iyy, 0.0f,
		0.0f, 0.0f, Izz
		});
}

Matrix3f Rigidbody::GetSphereInertiaTensorLocal()
{
	float radius = scale.x;

	float I = (2.0f / 5.0f) * (mass * radius * radius);

	return Matrix3f({
		I, 0.0f, 0.0f,
		0.0f, I, 0.0f,
		0.0f, 0.0f, I
		});
}

Matrix3f Rigidbody::GetTetrahedronInertiaTensorLocal()
{
	float s = scale.x;
	float I = (1.0f / 20.0f) * mass * s * s;

	return Matrix3f({
		I, 0.0f, 0.0f,
		0.0f, I, 0.0f,
		0.0f, 0.0f, I
		});
}

Matrix3f Rigidbody::GetInverseInertiaTensorWorld()
{
	Matrix3f iitLocal = inverseInertiaTensor;
	Quaternion q = rotation;
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

std::shared_ptr<BoundingSphere> Rigidbody::GetBoundingSphere()
{
	return m_boundingSphere;
}

BoundingBox Rigidbody::GetBoundingBox()
{
	return BoundingBox(position, scale);
}