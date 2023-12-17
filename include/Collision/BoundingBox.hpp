#pragma once

#include <Vector3.hpp>
#include <Collision/BoundingVolume.hpp>

class BoundingBox : public BoundingVolume
{
public:
	BoundingBox();
	BoundingBox(const Vector3f& center, const Vector3f& halfSize);
	BoundingBox(const BoundingBox& one, const BoundingBox& two);

	Vector3f GetHalfSize() const;
	bool Overlaps(std::shared_ptr<BoundingBox> other) const;

	Vector3f GetCenter() const override;
	float GetSize() const override;
	float GetGrowth(std::shared_ptr<BoundingVolume> other) const override;
private:
	Vector3f m_center;
	Vector3f m_halfSize;
	std::shared_ptr<BoundingVolume> m_other;
};