#include <Vector3.hpp>
#include <cmath>

template<typename T>
Vector3<T>::Vector3(T V) :
x(V),
y(V),
z(V)
{
}

template<typename T>
Vector3<T>::Vector3(T X, T Y, T Z) :
x(X),
y(Y),
z(Z)
{
}

template<typename T>
Vector3<T> Vector3<T>::operator+(const Vector3& vec) const
{
	return Vector3{ x + vec.x, y + vec.y, z + vec.z };
}

template<typename T>
Vector3<T> Vector3<T>::operator-(const Vector3& vec) const
{
	return Vector3{ x - vec.x, y - vec.y, z - vec.z };
}

template<typename T>
T Vector3<T>::operator*(const Vector3& vec) const
{
	return x * vec.x + y * vec.y + z * vec.z;
}

template<typename T>
Vector3<T> Vector3<T>::operator*(T value) const
{
	return Vector3{ x * value, y * value, z * value };
}

template<typename T>
Vector3<T> Vector3<T>::operator/(const Vector3& vec) const
{
	return Vector3{ x / vec.x, y / vec.y, z / vec.z };
}

template<typename T>
Vector3<T> Vector3<T>::operator/(T value) const
{
	return Vector3{ x / value, y / value, z / value };
}

template<typename T>
Vector3<T>& Vector3<T>::operator+=(const Vector3& vec)
{
	x += vec.x;
	y += vec.y;
	z += vec.z;

	return *this;
}

template<typename T>
Vector3<T>& Vector3<T>::operator-=(const Vector3& vec)
{
	x -= vec.x;
	y -= vec.y;
	z -= vec.z;

	return *this;
}

/*template<typename T>
Vector3<T>& Vector3<T>::operator*=(const Vector3& vec)
{
	x *= vec.x;
	y *= vec.y;
	z *= vec.z;

	return *this;
}*/

template<typename T>
Vector3<T>& Vector3<T>::operator*=(T value)
{
	x *= value;
	y *= value;
	z *= value;

	return *this;
}

template<typename T>
Vector3<T>& Vector3<T>::operator/=(const Vector3& vec)
{
	x /= vec.x;
	y /= vec.y;
	z /= vec.z;

	return *this;
}

template<typename T>
Vector3<T>& Vector3<T>::operator/=(T value)
{
	x /= value;
	y /= value;
	z /= value;

	return *this;
}

template<typename T>
float Vector3<T>::GetLength() const
{
	return std::sqrt(x*x + y*y + z*z);
}

template<typename T>
float Vector3<T>::GetLengthSquared() const
{
	return x*x + y*y + z*z;
}

template<typename T>
Vector3<T>& Vector3<T>::Normalize()
{
	*this = GetNormalized();
	return *this;
}

template<typename T>
Vector3<T>& Vector3<T>::UnitNormalize()
{
	*this = GetUnitNormalized();
	return *this;
}

template<typename T>
Vector3<T> Vector3<T>::GetNormalized() const
{
	return Vector3(*this / GetLength());
}

template<typename T>
Vector3<T> Vector3<T>::GetUnitNormalized() const
{
	Vector3<T> temp = *this / GetLength();
	return Vector3(temp / (temp.x + temp.y + temp.z));
}

template<typename T>
float Vector3<T>::DotProduct(const Vector3& vecA, const Vector3& vecB)
{
	return vecA.x*vecB.x + vecA.y*vecB.y + vecA.z*vecB.z;
}

template<typename T>
Vector3<T> Vector3<T>::CrossProduct(const Vector3& vecA, const Vector3& vecB)
{
	return Vector3(vecA.y*vecB.z - vecA.z*vecB.y,
				vecA.z*vecB.x - vecA.x*vecB.z,
				vecA.x*vecB.y - vecA.y*vecB.x);
}

template<typename T>
Vector3<T> Vector3<T>::Min(const Vector3& vecA, const Vector3& vecB)
{
	return Vector3{ std::min(vecA.x, vecB.x), std::min(vecA.y, vecB.y), std::min(vecA.z, vecB.z) };
}

template<typename T>
Vector3<T> Vector3<T>::Max(const Vector3& vecA, const Vector3& vecB)
{
	return Vector3{ std::max(vecA.x, vecB.x), std::max(vecA.y, vecB.y), std::max(vecA.z, vecB.z) };
}

template<typename T>
Vector3<T> Vector3<T>::Cross(const Vector3<T>& vecB) const
{
	return CrossProduct(*this, vecB);
}


template<typename T>
Vector3<T> operator*(T value, const Vector3<T>& vec)
{
	return Vector3{ vec.x * value, vec.y * value, vec.z * value };
}

template<typename T>
Vector3<T> operator/(T value, const Vector3<T>& vec)
{
	return Vector3{ vec.x / value, vec.y / value, vec.z / value };
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& vec)
{
	return os << "Vector3(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}

template<typename T>
const Vector3<T> Vector3<T>::Zero = Vector3<T>::GetZero();

template<typename T>
const Vector3<T> Vector3<T>::One = Vector3<T>::GetOne();