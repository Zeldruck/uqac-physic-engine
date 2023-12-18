#pragma once

#include <Vector3.hpp>
#include <Collision/BoundingVolume.hpp>

class BoundingSphere : public BoundingVolume
{
public:
	BoundingSphere();
	BoundingSphere(const Vector3f& center, float radius);
	BoundingSphere(std::shared_ptr<BoundingSphere> one, std::shared_ptr<BoundingSphere> two);

	float GetRadius() const;
	bool Overlaps(std::shared_ptr<BoundingSphere> other) const;

	Vector3f GetCenter() const override;
	float GetSize() const override;
	float GetGrowth(std::shared_ptr<BoundingVolume> other) const override;

private:
	Vector3f m_center;
	float m_radius;

	std::shared_ptr<BoundingSphere> m_other;
};