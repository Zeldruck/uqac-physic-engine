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
public:
	BVHNode(std::shared_ptr<Rigidbody> rigidbody, const TBoundingVolume& volume);
	BVHNode(std::shared_ptr<BVHNode<TBoundingVolume>> node);
	BVHNode(std::shared_ptr<BVHNode<TBoundingVolume>> node, std::shared_ptr<Rigidbody> body, const TBoundingVolume& volume);
	~BVHNode();

	bool IsLeaf() const;
	bool Overlaps(const std::shared_ptr<BVHNode<TBoundingVolume>> other) const;
	unsigned int GetPotentialContact(std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
	unsigned int BVHNode<TBoundingVolume>::GetPotentialContactsWith(const std::shared_ptr<BVHNode<BoundingVolume>> other, std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
	void Insert(std::shared_ptr<Rigidbody> newRigidbody, const TBoundingVolume& newVolume);
	void RecalculateBoundingVolume(bool recurse = true);

	std::array<std::shared_ptr<BVHNode>, 2> children;
	std::shared_ptr<BVHNode> m_parent;
	/* Single bounding volume encompassing all the children of this node*/
	TBoundingVolume m_volume;
	/* only leaf node can have a rigidbody
		Stock the rigidbody of this current node*/
	std::shared_ptr<Rigidbody> m_rigidbody;
};