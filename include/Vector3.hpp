#pragma once

#include <ostream>

template<typename T>
struct Vector3
{
	Vector3() = default;
	explicit Vector3(T V);
	Vector3(T X, T Y, T Z);

	Vector3 operator+(const Vector3& vec) const;
	Vector3 operator-(const Vector3& vec) const;
	Vector3 operator*(const Vector3& vec) const;
	Vector3 operator*(T value) const;
	Vector3 operator/(const Vector3& vec) const;
	Vector3 operator/(T value) const;

	Vector3& operator+=(const Vector3& vec);
	Vector3& operator-=(const Vector3& vec);
	Vector3& operator*=(const Vector3& vec);
	Vector3& operator*=(T value);
	Vector3& operator/=(const Vector3& vec);
	Vector3& operator/=(T value);

	float GetLength() const;
	Vector3& Normalize();
	Vector3& UnitNormalize();
	Vector3 GetNormalized() const;
	Vector3 GetUnitNormalized() const;

	static float DotProduct(const Vector3& vecA, const Vector3& vecB);
	static Vector3 CrossProduct(const Vector3& vecA, const Vector3& vecB);

	T x, y, z;
};

template<typename T> Vector3<T> operator*(T value, const Vector3<T>& vec);
template<typename T> Vector3<T> operator/(T value, const Vector3<T>& vec);

template<typename T> std::ostream& operator<<(std::ostream& os, const Vector3<T>& vec);

using Vector3f = Vector3<float>;
using Vector3i = Vector3<int>;

#include <Vector3.inl>