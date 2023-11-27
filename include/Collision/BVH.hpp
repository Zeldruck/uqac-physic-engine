#pragma once

#include <memory>
#include <array>

class Rigidbody;
class BoundingVolume;

struct PotentialContact
{
	/* Bodies that might be in contact */
	std::array<std::shared_ptr<Rigidbody>, 2> rigidbodies;
};

template<class TBoundingVolume>
class BVHNode
{
	std::array<std::shared_ptr<BVHNode>, 2> children;

	/* Single bounding volume encompassing all the children of this node*/
	TBoundingVolume volume;

	/* only leaf node can have a rigidbody 
		Stock the rigidbody of this current node*/
	std::shared_ptr<Rigidbody> rigidbody;

	inline bool IsLeaf() const;
	bool Overlaps(const std::shared_ptr<BVHNode<TBoundingVolume>> other) const;
	unsigned int GetPotentialContact(std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
	unsigned int BVHNode<TBoundingVolume>::GetPotentialContactsWith(const std::shared_ptr<BVHNode<BoundingVolume>> other, std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
};