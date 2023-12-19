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
	T operator*(const Vector3& vec) const;
	Vector3 operator*(T value) const;
	Vector3 operator/(const Vector3& vec) const;
	Vector3 operator/(T value) const;

	Vector3& operator+=(const Vector3& vec);
	Vector3& operator-=(const Vector3& vec);
	//Vector3& operator*=(const Vector3& vec);
	Vector3& operator*=(T value);
	Vector3& operator/=(const Vector3& vec);
	Vector3& operator/=(T value);

	float GetLength() const;
	float GetLengthSquared() const;
	Vector3& Normalize();
	Vector3& UnitNormalize();
	Vector3 GetNormalized() const;
	Vector3 GetUnitNormalized() const;
	Vector3 GetInvert() const;

	static float DotProduct(const Vector3& vecA, const Vector3& vecB);
	static Vector3 CrossProduct(const Vector3& vecA, const Vector3& vecB);
	static Vector3 Min(const Vector3& vecA, const Vector3& vecB);
	static Vector3 Max(const Vector3& vecA, const Vector3& vecB);


	Vector3 Cross(const Vector3<T>& vecB) const;

	static const Vector3 Zero;
	static const Vector3 One;
	static const Vector3 Up;
	static const Vector3 Down;
	static const Vector3 Left;
	static const Vector3 Right;
	static const Vector3 Forward;
	static const Vector3 Backward;
	T x, y, z;

private:
	static Vector3 GetZero() { return Vector3(0, 0, 0); }
	static Vector3 GetOne() { return Vector3(1, 1, 1); }
	static Vector3 GetUp() { return Vector3(0, 1, 0); }
	static Vector3 GetDown() { return Vector3(0, -1, 0); }
	static Vector3 GetLeft() { return Vector3(-1, 0, 0); }
	static Vector3 GetRight() { return Vector3(1, 0, 0); }
	static Vector3 GetForward() { return Vector3(0, 0, 1); }
};

template<typename T> Vector3<T> operator*(T value, const Vector3<T>& vec);
template<typename T> Vector3<T> operator/(T value, const Vector3<T>& vec);

template<typename T> std::ostream& operator<<(std::ostream& os, const Vector3<T>& vec);

using Vector3f = Vector3<float>;
using Vector3i = Vector3<int>;

#include <Vector3.inl>