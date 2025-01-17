#include <Matrix3.hpp>
#include <Constants/MathConstants.hpp>
#include <cmath>

template<typename T>
Matrix3<T>::Matrix3(std::array<T, 3 * 3> values) :
	m_values(values)
{
}

template<typename T>
T Matrix3<T>::Determinant() const
{
	// computes the inverse of a matrix m
	T det = Value(0, 0) * (Value(1, 1) * Value(2, 2) - Value(2, 1) * Value(1, 2)) -
		Value(0, 1) * (Value(1, 0) * Value(2, 2) - Value(1, 2) * Value(2, 0)) +
		Value(0, 2) * (Value(1, 0) * Value(2, 1) - Value(1, 1) * Value(2, 0));

	return det;
}

template<typename T>
Matrix3<T> Matrix3<T>::Inverse() const
{
	T invDet = 1.0f / Determinant();

	Matrix3 result;
	result(0, 0) = (Value(1, 1) * Value(2, 2) - Value(2, 1) * Value(1, 2)) * invDet;
	result(0, 1) = -(Value(0, 1) * Value(2, 2) - Value(0, 2) * Value(2, 1)) * invDet;
	result(0, 2) = (Value(0, 1) * Value(1, 2) - Value(0, 2) * Value(1, 1)) * invDet;
	result(1, 0) = -(Value(1, 0) * Value(2, 2) - Value(1, 2) * Value(2, 0)) * invDet;
	result(1, 1) = (Value(0, 0) * Value(2, 2) - Value(0, 2) * Value(2, 0)) * invDet;
	result(1, 2) = -(Value(0, 0) * Value(1, 2) - Value(1, 0) * Value(0, 2)) * invDet;
	result(2, 0) = (Value(1, 0) * Value(2, 1) - Value(2, 0) * Value(1, 1)) * invDet;
	result(2, 1) = -(Value(0, 0) * Value(2, 1) - Value(2, 0) * Value(0, 1)) * invDet;
	result(2, 2) = (Value(0, 0) * Value(1, 1) - Value(1, 0) * Value(0, 1)) * invDet;

	return result;
}

template<typename T>
Matrix3<T> Matrix3<T>::Transpose() const
{
	Matrix3 mat;
	for (std::size_t i = 0; i < 3; ++i)
	{
		for (std::size_t j = 0; j < 3; ++j)
			mat(j, i) = Value(i, j);
	}

	return mat;
}

template<typename T>
inline Vector3f Matrix3<T>::TransformTranspose(const Vector3f& vector) const
{
	return Vector3(
		vector.x * m_values[0] + vector.y * m_values[3] + vector.z * m_values[6],
		vector.x * m_values[1] + vector.y * m_values[4] + vector.z * m_values[7],
		vector.x * m_values[2] + vector.y * m_values[5] + vector.z * m_values[8]
	);
}

template<typename T>
T& Matrix3<T>::Value(std::size_t i, std::size_t j)
{
	return m_values[i * 3 + j];
}

template<typename T>
const T& Matrix3<T>::Value(std::size_t i, std::size_t j) const
{
	return m_values[i * 3 + j];
}

template<typename T>
T& Matrix3<T>::operator()(std::size_t i, std::size_t j)
{
	return Value(i, j);
}

template<typename T>
const T& Matrix3<T>::operator()(std::size_t i, std::size_t j) const
{
	return Value(i, j);
}

template<typename T>
Matrix3<T> Matrix3<T>::operator*(const Matrix3& rhs) const
{
	Matrix3 result;
	for (std::size_t i = 0; i < 3; ++i)
	{
		for (std::size_t j = 0; j < 3; ++j)
		{
			T sum = 0.0f;
			for (std::size_t k = 0; k < 3; ++k)
				sum += Value(i, k) * rhs(k, j);

			result(i, j) = sum;
		}
	}

	return result;
}

template<typename T>
Vector2<T> Matrix3<T>::operator*(const Vector2<T>& vec) const
{
	Vector2<T> result;
	result.x = Value(0, 0) * vec.x + Value(0, 1) * vec.y + Value(0, 2);
	result.y = Value(1, 0) * vec.x + Value(1, 1) * vec.y + Value(1, 2);

	return result;
}

template<typename T>
Vector3<T> Matrix3<T>::operator*(const Vector3<T>& vec) const
{
	Vector3<T> result;
	result.x = Value(0, 0) * vec.x + Value(0, 1) * vec.y + Value(0, 2) * vec.z;
	result.y = Value(1, 0) * vec.x + Value(1, 1) * vec.y + Value(1, 2) * vec.z;
	result.z = Value(2, 0) * vec.z + Value(2, 1) * vec.y + Value(2, 2) * vec.z;

	return result;
}

template<typename T>
Vector4<T> Matrix3<T>::operator*(const Vector4<T>& vec) const
{
	Vector4<T> result;
	result.x = Value(0, 0) * vec.x + Value(0, 1) * vec.y + Value(0, 2) * vec.z;
	result.y = Value(1, 0) * vec.x + Value(1, 1) * vec.y + Value(1, 2) * vec.z;
	result.z = Value(2, 0) * vec.z + Value(2, 1) * vec.y + Value(2, 2) * vec.z;

	return result;
}

template<typename T>
Matrix3<T> Matrix3<T>::Identity()
{
	return Matrix3({
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f
		});
}

template<typename T>
Matrix3<T> Matrix3<T>::Rotate(float degreeAngle)
{
	float sinAngle = std::sin(degreeAngle * Deg2Rad);
	float cosAngle = std::cos(degreeAngle * Deg2Rad);

	return Matrix3({
		cosAngle, -sinAngle, 0.0f,
		sinAngle,  cosAngle, 0.0f,
		0.0f,          0.0f, 1.0f
		});
}

template<typename T>
Matrix3<T> Matrix3<T>::Scale(const Vector2<T>& scale)
{
	return Matrix3({
		scale.x, 0.0f,    0.0f,
		0.0f,    scale.y, 0.0f,
		0.f,     0.0f,    1.0f
		});
}

template<typename T>
Matrix3<T> Matrix3<T>::Translate(const Vector2<T>& translation)
{
	return Matrix3({
		1.0f, 0.0f, translation.x,
		0.0f, 1.0f, translation.y,
		0.0f, 0.0f, 1.0f,
		});
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Matrix3<T>& mat)
{
	return os << "Matrix3(" << mat.Value(0, 0) << ", " << mat.Value(0, 1) << ", " << mat.Value(0, 2) << ",\n"
		<< "        " << mat.Value(1, 0) << ", " << mat.Value(1, 1) << ", " << mat.Value(1, 2) << ",\n"
		<< "        " << mat.Value(2, 0) << ", " << mat.Value(2, 1) << ", " << mat.Value(2, 2) << ")";
}