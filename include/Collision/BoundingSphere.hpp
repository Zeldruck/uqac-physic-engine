#pragma once

#include <Vector3.hpp>
#include "BoundingVolume.hpp"

template<class TBoundingVolume>
class BoundingSphere : public BoundingVolume
{
public:
	Vector3f center;
	float radius;

	std::shared_ptr<TBoundingVolume> volume;

	bool Overlaps(std::shared_ptr<TBoundingVolume> other) const override;
};