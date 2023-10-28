#pragma once

#include "Vector3.hpp"
#include "Matrix3.hpp"
#include <cmath>
#include <ostream>

template<typename T>
class Quaternion
{
public:
	Quaternion();
	Quaternion(T s);
	Quaternion(T s, T x, T y, T z);

	Quaternion(T angle, Vector3<T> unitAxis);

	void Set(T s, T x, T y, T z);

	inline T GetS();
	inline T GetX();
	inline T GetY();
	inline T GetZ();

	Quaternion operator+ (Quaternion q2) const;
	Quaternion operator- (Quaternion q2) const;
	Quaternion operator* (Quaternion q2) const;
	Quaternion operator/ (Quaternion q2) const;

	Quaternion conj();

	Quaternion& operator= (Quaternion rhs);
	Quaternion& operator= (T s_);
	int operator== (Quaternion rhs);
	int operator!= (Quaternion rhs);

	void Normalize();

	void MoveToRightHalfSphere();

	T Norm();
	T Norm2();

	void Quaternion2Matrix(Matrix3f& R);

	void GetRotation(T& angle, Vector3<T> unitAxis);

	void GetSinExponential(T& x, T& y, T& z);

protected:
	T s, x, y, z;

};

template<typename T> Quaternion<T> operator* (T alpha, const Quaternion<T>& q2);

template<typename T> std::ostream& operator<<(std::ostream& os, const Quaternion<T>& qua);

using Quaternionf = Quaternion<float>;

#include <Quaternion.inl>