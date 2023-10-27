#pragma once

#include <Vector2.hpp>
#include <Vector3.hpp>
#include <Vector4.hpp>
#include <array>
#include <ostream>

template<typename T>
class Matrix4
{
public:
	Matrix4() = default;
	Matrix4(std::array<T, 4 * 4> values);

	Matrix4 Inverse() const;

	Matrix4 Transpose() const;

	T& Value(std::size_t i, std::size_t j);
	const T& Value(std::size_t i, std::size_t j) const;

	T& operator()(std::size_t i, std::size_t j);
	const T& operator()(std::size_t i, std::size_t j) const;

	Matrix4 operator*(const Matrix4& rhs) const;
	Vector2<T> operator*(const Vector2<T>& vec) const;
	Vector3<T> operator*(const Vector3<T>& vec) const;
	Vector4<T> operator*(const Vector4<T>& vec) const;

	static Matrix4 Identity();
	static Matrix4 RotateAroundX(float degreeAngle);
	static Matrix4 RotateAroundY(float degreeAngle);
	static Matrix4 RotateAroundZ(float degreeAngle);
	static Matrix4 Scale(const Vector3<T>& scale);
	static Matrix4 Translate(const Vector3<T>& translation);

private:
	std::array<T, 4 * 4> m_values;
};

template<typename T> std::ostream& operator<<(std::ostream& os, const Matrix4<T>& mat);

using Matrix4f = Matrix4<float>;

#include <Matrix4.inl>