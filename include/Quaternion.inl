#pragma once

#include "Quaternion.hpp"
#include <Constants/MathConstants.hpp>
#include <cmath>

template <typename T>
Quaternion<T>::Quaternion()
{
	s = x = y = z = 0;
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
Quaternion<T> Quaternion<T>::operator- (Quaternion<T> q2) const
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
Quaternion<T> Quaternion<T>::conj()
{
	Quaternion<T> w(s, -x, -y, -z);
	return w;
}

template <typename T>
void Quaternion<T>::Quaternion2Matrix(Matrix3f& R)
{
	R[0] = 1 - 2 * y * y - 2 * z * z; R[1] = 2 * x * y - 2 * s * z;     R[2] = 2 * x * z + 2 * s * y;
	R[3] = 2 * x * y + 2 * s * z;     R[4] = 1 - 2 * x * x - 2 * z * z; R[5] = 2 * y * z - 2 * s * x;
	R[6] = 2 * x * z - 2 * s * y;     R[7] = 2 * y * z + 2 * s * x;     R[8] = 1 - 2 * x * x - 2 * y * y;
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
void Quaternion<T>::GetRotation(T& angle, Vector3<T> unitAxis)
{
	if ((s >= ((T)1)) || (s <= (T)(-1)))
	{
		angle = 0;
		unitAxis[0] = 1;
		unitAxis[0] = 0;
		unitAxis[0] = 0;
		return;
	}

	angle = 2.0 * acos(s);
	T sin2 = x * x + y * y + z * z;

	if (sin2 == 0)
	{
		unitAxis[0] = 1;
		unitAxis[0] = 0;
		unitAxis[0] = 0;
	}
	else
	{
		T inv = 1.0 / sqrt(sin2);
		unitAxis[0] = x * inv;
		unitAxis[1] = y * inv;
		unitAxis[2] = z * inv;
	}
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
	return Quaternion<T>(alpha * q2.s, alpha * q2.x, alpha * q2.y, alpha * q2.z);
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& qua)
{
	return os << "Quaternion(" << s << ", " << x << ", " << y << ", " << z << ")";
}