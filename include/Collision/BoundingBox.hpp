#pragma once

#include <Vector3.hpp>
#include "BoundingVolume.hpp"

class BoundingBox : public BoundingVolume
{
public:
	BoundingBox();
	BoundingBox(const Vector3f& center, const Vector3f& halfSize);
	BoundingBox(const BoundingBox& one, const BoundingBox& two);

	Vector3f GetCenter() const;
	Vector3f GetHalfSize() const;
	bool Overlaps(std::shared_ptr<BoundingBox> other) const;

	float GetSize() const override;
	float GetGrowth(const std::shared_ptr<BoundingVolume>& other) const override;
private:
	Vector3f m_center;
	Vector3f m_halfSize;
};