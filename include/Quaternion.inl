#pragma once

#include "Quaternion.hpp"
#include <Constants/MathConstants.hpp>
#include <cmath>

template <typename T>
Quaternion<T>::Quaternion()
{
	x = y = z = 0;
	s = 1;
}

template <typename T>
Quaternion<T>::Quaternion(T s_)
{
	s = s_;
	x = y = z = 0;
}

template <typename T>
Quaternion<T>::Quaternion(T angle, Vector3<T> unitAxis)
{
	s = cos(angle / 2.0);
	real sin2 = sin(angle / 2.0);
	x = sin2 * unitAxis[0];
	y = sin2 * unitAxis[1];
	z = sin2 * unitAxis[2];
}

template <typename T>
Quaternion<T>::Quaternion(T s_, T x_, T y_, T z_)
{
	s = s_;
	x = x_;
	y = y_;
	z = z_;
}

template <typename T>
void Quaternion<T>::Set(T s_g, T x_g, T y_g, T z_g) // sets quaternion to the new value
{
	s = s_g;
	x = x_g;
	y = y_g;
	z = z_g;
}

template <typename T>
T Quaternion<T>::GetS() { return s; }

template <typename T>
T Quaternion<T>::GetX() { return x; }

template <typename T>
T Quaternion<T>::GetY() { return y; }

template <typename T>
T Quaternion<T>::GetZ() { return z; }

template <typename T>
Quaternion<T>& Quaternion<T>::operator= (Quaternion<T> rhs)
{
	s = rhs.s;
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;

	return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator= (T s_g)
{
	s = s_g;
	x = 0;
	y = 0;
	z = 0;

	return *this;
}

template <typename T>
int Quaternion<T>::operator== (Quaternion<T> rhs)
{
	return ((s == rhs.s) && (x == rhs.x) &&
		(y == rhs.y) && (z == rhs.z));
}

template <typename T>
int Quaternion<T>::operator!= (Quaternion<T> rhs)
{
	return ((s != rhs.s) || (x != rhs.x) ||
		(y != rhs.y) || (z != rhs.z));
}

template <typename T>
void Quaternion<T>::Normalize()
{
	T invNorm;
	invNorm = (T)1.0 / (T)sqrt(Norm2());

	s *= invNorm;
	x *= invNorm;
	y *= invNorm;
	z *= invNorm;

}

template <typename T>
T Quaternion<T>::Norm2()
{
	return (s * s + x * x + y * y + z * z);
}

template <typename T>
T Quaternion<T>::Norm() { return sqrt(Norm2()); }

template <typename T>
Quaternion<T> Quaternion<T>::operator+ (Quaternion<T> q2) const
{
	Quaternion<T> w(s + q2.s, x + q2.x, y + q2.y, z + q2.z);

	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator+=(const Quaternion<T>& q2)
{
	Quaternion<T> w(s + q2.s, x + q2.x, y + q2.y, z + q2.z);
	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator- (Quaternion<T> q2) const
{
	Quaternion<T> w(s - q2.s, x - q2.x, y - q2.y, z - q2.z);
	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator-=(const Quaternion<T>& q2)
{
	Quaternion<T> w(s - q2.s, x - q2.x, y - q2.y, z - q2.z);
	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator* (Quaternion<T> q2) const
{
	Quaternion<T> w(
		s * q2.s - x * q2.x - y * q2.y - z * q2.z,
		s * q2.x + q2.s * x + y * q2.z - q2.y * z,
		s * q2.y + q2.s * y + q2.x * z - x * q2.z,
		s * q2.z + q2.s * z + x * q2.y - q2.x * y);

	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator*= (const Quaternion<T>& q2)
{
	Quaternion<T> w(
		s * q2.s - x * q2.x - y * q2.y - z * q2.z,
		s * q2.x + q2.s * x + y * q2.z - q2.y * z,
		s * q2.y + q2.s * y + q2.x * z - x * q2.z,
		s * q2.z + q2.s * z + x * q2.y - q2.x * y);

	return w;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator/ (Quaternion<T> q2) const
{
	Quaternion<T> invQ2;
	real invNorm2 = 1.0 / q2.Norm2();
	invQ2.s = q2.s * invNorm2;
	invQ2.x = -q2.x * invNorm2;
	invQ2.y = -q2.y * invNorm2;
	invQ2.z = -q2.z * invNorm2;

	return (*this * invQ2);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator/= (const Quaternion<T>& q2)
{
	Quaternion<T> invQ2;
	real invNorm2 = 1.0 / q2.Norm2();
	invQ2.s = q2.s * invNorm2;
	invQ2.x = -q2.x * invNorm2;
	invQ2.y = -q2.y * invNorm2;
	invQ2.z = -q2.z * invNorm2;

	return (*this * invQ2);
}

template <typename T>
Quaternion<T> Quaternion<T>::conj()
{
	Quaternion<T> w(s, -x, -y, -z);
	return w;
}

template <typename T>
void Quaternion<T>::QuaternionToMatrix(Matrix3f& R)
{
	R[0] = 1 - 2 * y * y - 2 * z * z; R[1] = 2 * x * y - 2 * s * z;     R[2] = 2 * x * z + 2 * s * y;
	R[3] = 2 * x * y + 2 * s * z;     R[4] = 1 - 2 * x * x - 2 * z * z; R[5] = 2 * y * z - 2 * s * x;
	R[6] = 2 * x * z - 2 * s * y;     R[7] = 2 * y * z + 2 * s * x;     R[8] = 1 - 2 * x * x - 2 * y * y;
}

template <typename T>
void Quaternion<T>::QuaternionToMatrix4(Matrix4<T>& R)
{
	R.Value(0, 0) = 1 - 2 * y * y - 2 * z * z; R.Value(0, 1) = 2 * x * y - 2 * s * z;     R.Value(0, 2) = 2 * x * z + 2 * s * y;	R.Value(0, 3) = 0.0f;
	R.Value(1, 0) = 2 * x * y + 2 * s * z;     R.Value(1, 1) = 1 - 2 * x * x - 2 * z * z; R.Value(1, 2) = 2 * y * z - 2 * s * x;	R.Value(1, 3) = 0.0f;
	R.Value(2, 0) = 2 * x * z - 2 * s * y;     R.Value(2, 1) = 2 * y * z + 2 * s * x;     R.Value(2, 2) = 1 - 2 * x * x - 2 * y * y;	R.Value(2, 3) = 0.0f;
	R.Value(3, 0) = 0;                         R.Value(3, 1) = 0;						  R.Value(3, 2) = 0.0f;						R.Value(3, 3) = 1.0f;
}

template <typename T>
void Quaternion<T>::GetSinExponential(T& seX, T& seY, T& seZ)
{
	if (s < 0)
	{
		seX = -x;
		seY = -y;
		seZ = -z;
	}
	else
	{
		seX = x;
		seY = y;
		seZ = z;
	}
}

template <typename T>
void Quaternion<T>::GetRotation(T& angle, Vector3<T>& unitAxis)
{
	if ((s >= ((T)1)) || (s <= (T)(-1)))
	{
		angle = 0;
		unitAxis = Vector3<T>(1, 0, 0);
		return;
	}

	angle = 2.0 * acos(s);
	T sin2 = x * x + y * y + z * z;

	if (sin2 == 0)
	{
		unitAxis = Vector3<T>(1, 0, 0);
	}
	else
	{
		T inv = 1.0 / sqrt(sin2);
		unitAxis = Vector3<T>(x, y, z) * inv;
	}
}

template<typename T>
Vector3<T> Quaternion<T>::GetRotation()
{
	Vector3<T> axis;
	T angle;

	this->GetRotation(angle, axis);

	return axis * angle;
}

template <typename T>
void Quaternion<T>::MoveToRightHalfSphere()
{
	if (s < 0)
	{
		s *= -1;
		x *= -1;
		y *= -1;
		z *= -1;
	}
}

template<typename T>
Quaternion<T> operator* (T alpha, Quaternion<T>& q2)
{
	return Quaternion<T>(alpha * q2.GetS(), alpha * q2.GetX(), alpha * q2.GetY(), alpha * q2.GetZ());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, Quaternion<T>& qua)
{
	return os << "Quaternion(" << qua.GetS() << ", " << qua.GetX() << ", " << qua.GetY() << ", " << qua.GetZ() << ")";
}

template<typename T>
void Quaternion<T>::RotateByVector(const Vector3<T>& v)
{
	Quaternion<T> q(0, v.x, v.y, v.z);
	(*this) *= q;
}

template<typename T>
void Quaternion<T>::AddScaleVector(const Vector3<T>& v, T scale)
{
	Quaternion<T> q(0, v.x * scale, v.y * scale, v.z * scale);
	q *= (*this);
	s += q.s * ((T)0.5);
	x += q.x * ((T)0.5);
	y += q.y * ((T)0.5);
	z += q.z * ((T)0.5);
}