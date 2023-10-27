#pragma once

#include <ostream>

template<typename T>
struct Vector2
{
	Vector2() = default;
	explicit Vector2(T V);
	Vector2(T X, T Y);

	Vector2 operator+(const Vector2& vec) const;
	Vector2 operator-(const Vector2& vec) const;
	Vector2 operator*(const Vector2& vec) const;
	Vector2 operator*(T value) const;
	Vector2 operator/(const Vector2& vec) const;
	Vector2 operator/(T value) const;

	Vector2& operator+=(const Vector2& vec);
	Vector2& operator-=(const Vector2& vec);
	Vector2& operator*=(const Vector2& vec);
	Vector2& operator*=(T value);
	Vector2& operator/=(const Vector2& vec);
	Vector2& operator/=(T value);

	static Vector2 Rotate(const Vector2& vec, float degrees);

	T x, y;
};

template<typename T> Vector2<T> operator*(T value, const Vector2<T>& vec);
template<typename T> Vector2<T> operator/(T value, const Vector2<T>& vec);

template<typename T> std::ostream& operator<<(std::ostream& os, const Vector2<T>& vec);

using Vector2f = Vector2<float>;
using Vector2i = Vector2<int>;

#include <Vector2.inl>