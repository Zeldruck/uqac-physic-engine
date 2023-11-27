#include <Collision/BoundingBox.hpp>

template<class TBoundingVolume>
bool BoundingBox<TBoundingVolume>::Overlaps(std::shared_ptr<TBoundingVolume> other) const
{
	return false;
}