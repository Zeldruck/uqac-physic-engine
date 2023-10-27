#pragma once

#include <ostream>

template<typename T>
struct Vector4
{
	Vector4() = default;
	explicit Vector4(T V);
	Vector4(T X, T Y, T Z, T W);

	Vector4 operator+(const Vector4& vec) const;
	Vector4 operator-(const Vector4& vec) const;
	Vector4 operator*(const Vector4& vec) const;
	Vector4 operator*(T value) const;
	Vector4 operator/(const Vector4& vec) const;
	Vector4 operator/(T value) const;

	Vector4& operator+=(const Vector4& vec);
	Vector4& operator-=(const Vector4& vec);
	Vector4& operator*=(const Vector4& vec);
	Vector4& operator*=(T value);
	Vector4& operator/=(const Vector4& vec);
	Vector4& operator/=(T value);

	T x, y, z, w;
};

template<typename T> Vector4<T> operator*(T value, const Vector4<T>& vec);
template<typename T> Vector4<T> operator/(T value, const Vector4<T>& vec);

template<typename T> std::ostream& operator<<(std::ostream& os, const Vector4<T>& vec);

using Vector4f = Vector4<float>;
using Vector4i = Vector4<int>;

#include <Vector4.inl>