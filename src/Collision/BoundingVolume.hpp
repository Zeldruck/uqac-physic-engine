#pragma once

#include <memory>

class BoundingVolume
{
public:
	virtual bool Overlaps(std::shared_ptr<BoundingVolume> other) const = 0;
};