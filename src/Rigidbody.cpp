#include "Rigidbody.hpp"
#include "Constants/PhysicConstants.hpp"
#include "Constants/MathConstants.hpp"
#include "Matrix4.hpp"
#include "Matrix3.hpp"
#include "Quaternion.hpp"

Rigidbody::Rigidbody()
	:
	transform(Transform()),
	velocity(Vector3f::Zero),
	m_acceleration(Vector3f::Zero),
	force(Vector3f::Zero),
	angularVelocity(Vector3f::Zero),
	m_angularAcceleration(Vector3f::Zero),
	//momentOfInertia(Vector3f::Zero),
	inertiaTensor(GetBoxInertiaTensorWorld()),
	mass(MIN_MASS)
{
	CalculateDerivedData();
}

Rigidbody::Rigidbody(Transform transform, Vector3f velocity, Vector3f acceleration, float mass, Vector3f angularVelocity, Vector3f angularAcceleration, Vector3f momentOfInertia, std::string name)
	:
	transform(transform),
	velocity(velocity),
	m_acceleration(acceleration),
	force(Vector3f::Zero),
	mass(mass > MIN_MASS ? mass : MIN_MASS),
	angularVelocity(angularVelocity),
	m_angularAcceleration(angularAcceleration),
	//momentOfInertia(momentOfInertia),
	inertiaTensor(GetBoxInertiaTensorWorld()),
	name(name)
{
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

void Rigidbody::RemoveForce(const Vector3f& f)
{
	force -= f;
}

Vector3f Rigidbody::GetPointInWorldSpace(const Vector3f& point)
{
	return transformMatrix * point;
}

//void Rigidbody::AddTorque(const Vector3f& t)
//{
//	torque += t;
//}
//
//void Rigidbody::RemoveTorque(const Vector3f& t)
//{
//	torque -= t;
//}

void Rigidbody::AddForceAtPoint(const Vector3f& f, const Vector3f& point)
{
	force += f;
	torque += (point - transform.position).Cross(f);
}

void Rigidbody::AddForceAtBodyPoint(const Vector3f& f, const Vector3f& point)
{
	force += f;
	torque += point.Cross(f);
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
	m_acceleration = force / mass;
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
	transformMatrix = Matrix4f::Identity();
	transformMatrix = transformMatrix * Matrix4f::Scale(transform.scale);
	Quaternionf q = transform.rotation;
	q.QuaternionToMatrix4(transformMatrix);
	//Matrix3f transformMatrix3 = transformMatrix.GetSubmatrix3(0, 0);
	//q.QuaternionToMatrix(transformMatrix3);
	//transformMatrix = transformMatrix * Matrix4f::RotateAroundX(transform.rotation.GetX());
	//transformMatrix = transformMatrix * Matrix4f::RotateAroundY(transform.rotation.GetY());
	//transformMatrix = transformMatrix * Matrix4f::RotateAroundZ(transform.rotation.GetZ());
	//transformMatrix = transformMatrix * Matrix4f::Rotate(transform.rotation);
	transformMatrix = transformMatrix * Matrix4f::Translate(transform.position);
}

void Rigidbody::CalculateDerivedData()
{
	//transform.rotation.Normalize();
	CalculateTransformMatrix();
	inverseMass = 1.f / mass;
	CalculateInverseInertiaTensor();
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
	Matrix3f inertiaTensorLocal = GetBoxInertiaTensorLocal();

	// Create a 4x4 identity matrix with the upper-left 3x3 block containing the inertiaTensorLocal
	Matrix4f inertiaTensorLocal4x4 = Matrix4f::Identity();
	inertiaTensorLocal4x4.SetSubmatrix3(0, 0, inertiaTensorLocal);

	// Calculate the inertiaTensorWorld using the full transformation matrix
	Matrix4f transformMatrixTranspose = transformMatrix.Transpose();
	Matrix4f inertiaTensorWorld4x4 = transformMatrix * inertiaTensorLocal4x4 * transformMatrixTranspose;

	// Extract the upper-left 3x3 block as the final inertiaTensorWorld
	Matrix3f inertiaTensorWorld = inertiaTensorWorld4x4.GetSubmatrix3(0, 0);

	return inertiaTensorWorld;
}


Matrix3f Rigidbody::GetSphereInertiaTensorLocal()
{
		float mass = this->mass;
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
	Matrix3f inertiaTensorLocal = GetSphereInertiaTensorLocal();
	//Matrix3f inertiaTensorWorld = tranformMatrix.GetBasisMatrix() * inertiaTensorLocal * tranformMatrix.GetBasisMatrix().Transpose();
	//return inertiaTensorWorld;
	return inertiaTensorLocal;

}

void Rigidbody::SetInertiaTensor(const Matrix3f& _inertiaTensor)
{
	inertiaTensor = _inertiaTensor;
}

void Rigidbody::CalculateInverseInertiaTensor()
{
	inverseInertiaTensor = inertiaTensor.Inverse();
}