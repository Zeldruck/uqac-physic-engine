#include <Collision/BoundingSphere.hpp>

template<class TBoundingVolume>
bool BoundingSphere<TBoundingVolume>::Overlaps(std::shared_ptr<TBoundingVolume> other) const
{
	return false;
}