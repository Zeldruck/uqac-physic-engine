#pragma once

#include <Vector3.hpp>
#include <Collision/BoundingVolume.hpp>

class BoundingSphere : public BoundingVolume
{
public:
	BoundingSphere();
	BoundingSphere(const Vector3f& center, float radius);
	BoundingSphere(const BoundingSphere& one, const BoundingSphere& two);

	Vector3f GetCenter() const;
	float GetRadius() const;
	bool Overlaps(std::shared_ptr<BoundingSphere> other) const;

	float GetSize() const override;
	float GetGrowth(const std::shared_ptr<BoundingVolume>& other) const override;

private:
	Vector3f m_center;
	float m_radius;
};