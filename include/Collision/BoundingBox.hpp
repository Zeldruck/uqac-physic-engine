#pragma once

#include <Vector3.hpp>
#include "BoundingVolume.hpp"

template <class TBoundingVolume>
class BoundingBox : public BoundingVolume
{
public:
	Vector3f center;
	Vector3f halfSize;

	std::shared_ptr<TBoundingVolume> volume;

	bool Overlaps(std::shared_ptr<TBoundingVolume> other) const override;
};