#pragma once

#include <memory>
#include <Vector3.hpp>

class BoundingVolume
{
public:
	virtual float GetSize() const = 0;
	virtual float GetGrowth(std::shared_ptr<BoundingVolume> other) const = 0;
	virtual Vector3f GetCenter() const = 0;
};