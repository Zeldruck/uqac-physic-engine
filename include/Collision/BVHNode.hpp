#pragma once

#include <memory>
#include <array>
#include <Collision/BoundingSphere.hpp>

class Rigidbody;
class Primitive;
class Sphere;
class Box;

struct PotentialContact
{
public:
	/* Bodies that might be in contact */
	std::array<std::shared_ptr<Rigidbody>, 2> rigidbodies;
};

class BVHNode
{
public:
	BVHNode(std::shared_ptr<Rigidbody> rigidbody);
	BVHNode(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<BoundingSphere> volume);
	BVHNode(std::shared_ptr<BVHNode> node);
	BVHNode(std::shared_ptr<BVHNode> node, std::shared_ptr<Rigidbody> body, std::shared_ptr<BoundingSphere> volume);
	BVHNode(std::shared_ptr<Primitive> primitive);
	BVHNode(std::shared_ptr<Primitive> primitive, std::shared_ptr<BoundingSphere> volume);
	BVHNode(std::shared_ptr<Sphere> sphere, std::shared_ptr<BoundingSphere> volume);
	BVHNode(std::shared_ptr<Box> box, std::shared_ptr<BoundingSphere> volume);

	bool IsLeaf() const;
	bool Overlaps(std::shared_ptr<BVHNode> other) const;
	unsigned int GetPotentialContact(PotentialContact* contacts, unsigned int limit) const;
	unsigned int GetPotentialContactsWith(std::shared_ptr<BVHNode> other, PotentialContact* contacts, unsigned int limit) const;
	void Insert(std::shared_ptr<Rigidbody> newRigidbody, std::shared_ptr<BoundingSphere> newVolume);
	void Insert(std::shared_ptr<Primitive> newPrimitive, std::shared_ptr<BoundingSphere> newVolume);
	void Insert(std::shared_ptr<Sphere> newSphere, std::shared_ptr<BoundingSphere> volume);
	void Insert(std::shared_ptr<Box> newBox, std::shared_ptr<BoundingSphere> volume);

	void RecalculateBoundingVolume(bool recurse = true);
	std::shared_ptr<BVHNode> GetRoot();

	std::array<std::shared_ptr<BVHNode>, 2> children;
	std::shared_ptr<BVHNode> m_parent;
	/* Single bounding volume encompassing all the children of this node*/
	std::shared_ptr<BoundingSphere> m_volume;
	/* only leaf node can have a rigidbody
			Stock the rigidbody of this current node*/
	std::shared_ptr<Rigidbody> m_rigidbody;

};