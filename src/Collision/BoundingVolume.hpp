#pragma once

#include <memory>

template<class TBoundingVolume>
class BoundingVolume
{
public:
	virtual bool Overlaps(std::shared_ptr<TBoundingVolume> other) const = 0;
};