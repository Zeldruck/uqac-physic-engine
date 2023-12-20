#pragma once

#include <memory>
#include <Vector3.hpp>

class BoundingVolume
{
public:
	BoundingVolume() = default;

	virtual float GetSize() const;
	virtual float GetGrowth(std::shared_ptr<BoundingVolume> other) const;
	virtual Vector3f GetCenter() const;
};