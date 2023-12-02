#pragma once

#include <memory>

class BoundingVolume
{
public:
	virtual float GetSize() const = 0;
	virtual float GetGrowth(const std::shared_ptr<BoundingVolume>& other) const = 0;
};