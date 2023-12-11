#include <Collision/BoundingBox.hpp>

BoundingBox::BoundingBox() :
	m_center(Vector3f(0.0f, 0.0f, 0.0f)),
	m_halfSize(Vector3f(0.0f, 0.0f, 0.0f))
{
}

BoundingBox::BoundingBox(const Vector3f& m_center, const Vector3f& m_halfSize) :
	m_center(m_center),
	m_halfSize(m_halfSize)
{
}

BoundingBox::BoundingBox(const BoundingBox& one, const BoundingBox& two)
{
	Vector3f minOne = one.m_center - one.m_halfSize;
	Vector3f minTwo = two.m_center - two.m_halfSize;
	Vector3f maxOne = one.m_center + one.m_halfSize;
	Vector3f maxTwo = two.m_center + two.m_halfSize;

	Vector3f minVector = Vector3f::Min(minOne, minTwo);
	Vector3f maxVector = Vector3f::Max(maxOne, maxTwo);

	m_center = (minVector + maxVector) * 0.5f;
	m_halfSize = (maxVector - minVector) * 0.5f;
}

bool BoundingBox::Overlaps(std::shared_ptr<BoundingBox> other) const
{
	if (abs(m_center.x - other->m_center.x) > (m_halfSize.x + other->m_halfSize.x))
	{
		return false;
	}
	if (abs(m_center.y - other->m_center.y) > (m_halfSize.y + other->m_halfSize.y))
	{
		return false;
	}
	if (abs(m_center.z - other->m_center.z) > (m_halfSize.z + other->m_halfSize.z))
	{
		return false;
	}
	return true;
}

Vector3f BoundingBox::GetCenter() const
{
	return m_center;
}

Vector3f BoundingBox::GetHalfSize() const
{
	return m_halfSize;
}

float BoundingBox::GetSize() const
{
	return m_halfSize.x * 2.0f;
}

float BoundingBox::GetGrowth(std::shared_ptr<BoundingVolume> other) const
{
	auto box = std::dynamic_pointer_cast<BoundingBox>(other);
	Vector3f newHalf = Vector3f::Max(m_halfSize, box->m_halfSize);
	return (newHalf.x * 2.0f) - GetSize();
}