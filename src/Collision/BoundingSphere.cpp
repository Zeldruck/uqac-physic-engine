#include <Collision/BoundingSphere.hpp>
#include <Constants/MathConstants.hpp>
BoundingSphere::BoundingSphere() :
	m_center(Vector3f::Zero),
	m_radius(0.0f)
{
}

BoundingSphere::BoundingSphere(const Vector3f& center, float radius) :
	m_center(center),
	m_radius(radius)
{
}

BoundingSphere::BoundingSphere(std::shared_ptr<BoundingSphere> one, std::shared_ptr<BoundingSphere> two)
{
	Vector3f centerOffset = two->m_center - one->m_center;
	float distance = centerOffset.GetLength();
	float radiusDiff = two->m_radius - one->m_radius;

	// Check if the larger sphere encloses the small one
	if (radiusDiff * radiusDiff >= distance)
	{
		if (one->m_radius > two->m_radius)
		{
			m_center = one->m_center;
			m_radius = one->m_radius;
		}
		else
		{
			m_center = two->m_center;
			m_radius = two->m_radius;
		}
	}
	// Otherwise we need to work with partially overlapping spheres
	else
	{
		distance = sqrt(distance);
		// The new radius is a combination of both
		m_radius = (distance + one->m_radius + two->m_radius) * 0.5f;

		// The new center is an interpolation of both
		m_center = one->m_center;
		if (distance > 0.0f)
		{
			m_center += centerOffset * ((m_radius - one->m_radius) / distance);
		}
	}
}

bool BoundingSphere::Overlaps(std::shared_ptr<BoundingSphere> other) const
{
	float distanceSquared = (m_center - other->m_center).GetLengthSquared();
	return distanceSquared < (m_radius + other->m_radius) * (m_radius + other->m_radius);
}

Vector3f BoundingSphere::GetCenter() const
{
	return m_center;
}

float BoundingSphere::GetRadius() const
{
	return m_radius;
}

float BoundingSphere::GetSize() const
{
	return (4.0f / 3.0f) * PI * m_radius * m_radius * m_radius;
}

float BoundingSphere::GetGrowth(std::shared_ptr<BoundingVolume> other) const
{
	auto sphere = std::dynamic_pointer_cast<BoundingSphere>(other);
	Vector3f centerOffset = m_center - sphere->m_center;
	float distance = centerOffset.GetLength();
	return (distance + m_radius + sphere->m_radius) - m_radius;
}