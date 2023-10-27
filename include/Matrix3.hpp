#pragma once

#include <Vector2.hpp>
#include <array>
#include <ostream>

template<typename T>
class Matrix3
{
public:
	Matrix3() = default;
	Matrix3(std::array<T, 3 * 3> values);

	T Determinant() const;

	Matrix3 Inverse() const;

	Matrix3 Transpose() const;

	T& Value(std::size_t i, std::size_t j);
	const T& Value(std::size_t i, std::size_t j) const;

	T& operator()(std::size_t i, std::size_t j);
	const T& operator()(std::size_t i, std::size_t j) const;

	Matrix3 operator*(const Matrix3& rhs) const;
	Vector2<T> operator*(const Vector2<T>& vec) const;

	static Matrix3 Identity();
	static Matrix3 Rotate(float degreeAngle);
	static Matrix3 Scale(const Vector2<T>& scale);
	static Matrix3 Translate(const Vector2<T>& translation);

private:
	std::array<T, 3 * 3> m_values;
};

template<typename T> std::ostream& operator<<(std::ostream& os, const Matrix3<T>& mat);

using Matrix3f = Matrix3<float>;

#include <Matrix3.inl>