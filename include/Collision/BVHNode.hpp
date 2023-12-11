#pragma once

#include <memory>
#include <array>

class Rigidbody;

struct PotentialContact
{
	/* Bodies that might be in contact */
	std::array<std::shared_ptr<Rigidbody>, 2> rigidbodies;
};


template<class T>
class BVHNode
{
public:
	BVHNode(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<T> volume);
	BVHNode(std::shared_ptr<BVHNode<T>> node);
	BVHNode(std::shared_ptr<BVHNode<T>> node, std::shared_ptr<Rigidbody> body, const T& volume);
	//~BVHNode();

	bool IsLeaf() const;
	bool Overlaps(const std::shared_ptr<BVHNode<T>> other) const;
	//unsigned int GetPotentialContact(std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
	//unsigned int GetPotentialContactsWith(std::shared_ptr<BVHNode<T>> other, std::shared_ptr<PotentialContact> contacts, unsigned int limit) const;
	//void Insert(std::shared_ptr<Rigidbody> newRigidbody, const T& newVolume);
	void RecalculateBoundingVolume(bool recurse = true);

	std::array<std::shared_ptr<BVHNode>, 2> children;
	std::shared_ptr<BVHNode> m_parent;
	/* Single bounding volume encompassing all the children of this node*/
	std::shared_ptr<T> m_volume;
	/* only leaf node can have a rigidbody
		Stock the rigidbody of this current node*/
	std::shared_ptr<Rigidbody> m_rigidbody;
};

//class BVHNodeBox
//{
//public:
//	BVHNodeBox(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<BoundingBox> box);
//
//	std::shared_ptr<Rigidbody> m_rigidbody;
//	std::shared_ptr<BoundingBox> m_volume;
//};

#include "BVHNode.inl"