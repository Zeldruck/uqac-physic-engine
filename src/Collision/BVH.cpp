#include <Collision/BVH.hpp>

template<class TBoundingVolume>
bool BVHNode<TBoundingVolume>::IsLeaf() const
{
	return (rigidbody != nullptr);
}

template<class TBoundingVolume>
bool BVHNode<TBoundingVolume>::Overlaps(const std::shared_ptr<BVHNode<TBoundingVolume>> other) const
{
	return volume->Overlaps(other->volume);
}

template<class TBoundingVolume>
unsigned int BVHNode<TBoundingVolume>::GetPotentialContact(std::shared_ptr<PotentialContact> contacts, unsigned int limit) const
{
	if (isLeaf() || limit == 0)
		return 0;

	return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
}

template<class TBoundingVolume>
unsigned int BVHNode<TBoundingVolume>::GetPotentialContactsWith(const std::shared_ptr<BVHNode<BoundingVolume>> other, std::shared_ptr<PotentialContact> contacts, unsigned int limit) const
{
	if (!Overlaps(other) || limit == 0)
		return 0;

	if (IsLeaf() && other->IsLeaf())
	{
		contacts->rigidbodies[0] = rigidbody;
		contacts->rigidbodies[1] = other->body;
		return 1;
	}

	if (other->IsLeaf() ||
		(!IsLeaf() && volume->GetSize() >= other->volume->GetSize()))
	{
		unsigned int count = children[0]->GetPotentialContactsWith(other, contacts, limit);
	
		if (limit > count)
		{
			return count + children[1]->GetPotentialContactsWith(other, contacts + count, limit->count);
		}
		else
		{
			return count;
		}
	}
	else
	{
		unsigned int count = GetPotentialContactsWith(other->children[0], contacts, limit);
		if (limit > count)
		{
			return count + GetPotentialContactsWith(other->children[1], contacts + count, limit - count);
		}
		else
		{
			return count;
		}
	}
}