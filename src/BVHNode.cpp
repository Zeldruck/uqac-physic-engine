#include "Collision/BVHNode.hpp"
#include <memory>
#include <array>
#include <Collision/BoundingSphere.hpp>
#include <Collision/BoundingBox.hpp>
#include <Rigidbody.hpp>

BVHNode::BVHNode(std::shared_ptr<Rigidbody> rigidbody) :
	m_rigidbody(rigidbody),
	m_volume(rigidbody->m_boundingSphere),
	m_parent(nullptr)
{
}

BVHNode::BVHNode(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<BoundingSphere> volume) :
	m_rigidbody(rigidbody),
	m_volume(volume),
	m_parent(nullptr)
{
}

BVHNode::BVHNode(std::shared_ptr<BVHNode> node) :
	children(node->children),
	m_volume(node->m_volume),
	m_rigidbody(node->m_rigidbody),
	m_parent(node->m_parent)
{
}

BVHNode::BVHNode(std::shared_ptr<BVHNode> node, std::shared_ptr<Rigidbody> body, std::shared_ptr<BoundingSphere> volume)
	:
	children(node->children),
	m_volume(volume),
	m_rigidbody(body),
	m_parent(node)
{
}

bool BVHNode::IsLeaf() const
{
	return (m_rigidbody != nullptr);
}

bool BVHNode::Overlaps(std::shared_ptr<BVHNode> other) const
{
	return m_volume->Overlaps(other->m_volume);
}

unsigned int BVHNode::GetPotentialContact(PotentialContact* contacts, unsigned int limit) const
{
	if (IsLeaf() || limit == 0)
		return 0;

	return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
}

unsigned int BVHNode::GetPotentialContactsWith(std::shared_ptr<BVHNode> other, PotentialContact* contacts, unsigned int limit) const
{
	if (!Overlaps(other) || limit == 0)
		return 0;

	if (IsLeaf() && other->IsLeaf())
	{
		contacts->rigidbodies[0] = m_rigidbody;
		contacts->rigidbodies[1] = other->m_rigidbody;
		return 1;
	}

	if (other->IsLeaf() || (!IsLeaf() && m_volume->GetSize() >= other->m_volume->GetSize()))
	{
		unsigned int count = children[0]->GetPotentialContactsWith(other, contacts, limit);

		if (limit > count)
			return count + children[1]->GetPotentialContactsWith(other, contacts + count, limit - count);
		 else
			return count;
	}
	else
	{
		unsigned int count = GetPotentialContactsWith(other->children[0], contacts, limit);

		if (limit > count)
			return count + GetPotentialContactsWith(other->children[1], contacts + count, limit - count);
		else
			return count;
	}
}

void BVHNode::Insert(std::shared_ptr<Rigidbody> newRigidbody, std::shared_ptr<BoundingSphere> newVolume)
{
	if (IsLeaf())
	{
		// Children 0 has this node as parent but has data from this node
		children[0] = std::make_shared<BVHNode>(std::make_shared<BVHNode>(*this), m_rigidbody, m_volume);
		// Children 1 his new node with this node as parent
		children[1] = std::make_shared<BVHNode>(std::make_shared<BVHNode>(*this), newRigidbody, newVolume);

		m_rigidbody = nullptr;
		RecalculateBoundingVolume();
	}
	else
	{
		if (children[0]->m_volume->GetGrowth(newVolume) < children[1]->m_volume->GetGrowth(newVolume))
			children[0]->Insert(newRigidbody, newVolume);
		else
			children[1]->Insert(newRigidbody, newVolume);
	}
}

void BVHNode::RecalculateBoundingVolume(bool recurse)
{
	if (IsLeaf())
		return;

	m_volume = std::make_shared<BoundingSphere>(children[0]->m_volume, children[1]->m_volume);

	if (m_parent != nullptr)
		m_parent->RecalculateBoundingVolume(true);
}

std::shared_ptr<BVHNode> BVHNode::GetRoot()
{
	if (m_parent == nullptr)
		return std::make_shared<BVHNode>(*this);
	else
		return m_parent->GetRoot();
}