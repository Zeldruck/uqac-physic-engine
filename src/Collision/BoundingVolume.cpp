#include <Collision/BoundingVolume.hpp>

float BoundingVolume::GetSize() const
{
	return 0.0f;
}

float BoundingVolume::GetGrowth(std::shared_ptr<BoundingVolume> other) const
{
	return 0.0f;
}

Vector3f BoundingVolume::GetCenter() const
{
	return Vector3f::Zero;
}