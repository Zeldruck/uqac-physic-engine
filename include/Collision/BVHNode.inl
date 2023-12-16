#include <Collision/BVHNode.hpp>
#include <Collision/BoundingVolume.hpp>
#include <Collision/BoundingSphere.hpp>
#include <Collision/BoundingBox.hpp>
#include <Rigidbody.hpp>

template<typename T>
BVHNode<T>::BVHNode(std::shared_ptr<Rigidbody> rigidbody, std::shared_ptr<T> volume) :
	m_rigidbody(rigidbody),
	m_volume(volume),
	m_parent(nullptr)
{
}

template<typename T>
BVHNode<T>::BVHNode(std::shared_ptr<BVHNode<T>> node) :
	children(node->children),
	m_volume(node->volume),
	m_rigidbody(node->rigidbody),
	m_parent(node->parent)
{
}

template<typename T>
BVHNode<T>::BVHNode(std::shared_ptr<BVHNode<T>> node, std::shared_ptr<Rigidbody> body, const T& volume) :
	children(node->children),
	m_volume(node->volume),
	m_rigidbody(node->rigidbody),
	m_parent(node->parent)
{
}

template<typename T>
bool BVHNode<T>::IsLeaf() const
{
	return (m_rigidbody != nullptr);
}

template<typename T>
bool BVHNode<T>::Overlaps(const std::shared_ptr<BVHNode<T>> other) const
{
	return m_volume->Overlaps(other->volume);
}

template<typename T>
unsigned int BVHNode<T>::GetPotentialContact(std::shared_ptr<PotentialContact> contacts, unsigned int limit) const
{
	if (isLeaf() || limit == 0)
		return 0;

	return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
}

template<typename T>
unsigned int BVHNode<T>::GetPotentialContactsWith(std::shared_ptr<BVHNode<T>> other, std::shared_ptr<PotentialContact> contacts, unsigned int limit) const
{
	if (!Overlaps(other) || limit == 0)
		return 0;

	if (IsLeaf() && other->IsLeaf())
	{
		contacts->rigidbodies[0] = m_rigidbody;
		contacts->rigidbodies[1] = other->body;
		return 1;
	}

	// Determines which node to descend into first.
	// If either is a leaf, then we descend the other.
	// If both are branches, then we use the size to descend.
	if (other->IsLeaf() || (!IsLeaf() && m_volume->GetSize() >= other->volume->GetSize()))
	{
		// Recurse into ourself
		unsigned int count = children[0]->GetPotentialContactsWith(other, contacts, limit);

		// Check we have enough slots to do the other side too
		if (limit > count)
		{
			return count + children[1]->GetPotentialContactsWith(other, contacts + count, limit - count);
		}
		else
		{
			// Not enough slots. Store the parent and stop
			return count;
		}
	}
	else
	{
		// Recurse into the other node
		unsigned int count = GetPotentialContactsWith(other->children[0], contacts, limit);

		// Check we have enough slots to do the other side too
		if (limit > count)
		{
			return count + GetPotentialContactsWith(other->children[1], contacts + count, limit - count);
		}
		else
		{
			// Not enough slots. Store the parent and stop
			return count;
		}
	}
}

template <typename T>
void BVHNode<T>::Insert(std::shared_ptr<Rigidbody> newRigidbody, const T& newVolume)
{
	// If we are a leaf, then the only option is to spawn two new children and place the new body in one.
	if (IsLeaf())
	{
		// Child one is a copy of us.
		children[0] = std::make_shared<BVHNode<T>>(*this);

		// Child two holds the new body
		children[1] = std::make_shared<BVHNode<T>>(*this, newRigidbody, newVolume);

		// And we now loose the body (we're no longer a leaf)
		this->m_rigidbody = nullptr;

		// We need to recalculate our bounding volume
		RecalculateBoundingVolume();
	}
	// Otherwise we need to work out which child gets to keep the inserted body.
	// We give it to whoever would grow the least to incorporate it.
	else
	{
		// Work out which child would grow least
		float growth0 = children[0]->volume->GetGrowth(newVolume);
		float growth1 = children[1]->volume->GetGrowth(newVolume);

		if (growth0 < growth1)
		{
			children[0]->Insert(newRigidbody, newVolume);
		}
		else
		{
			children[1]->Insert(newRigidbody, newVolume);
		}
	}
}

template <typename T>
void BVHNode<T>::RecalculateBoundingVolume(bool recurse /* = true */)
{
	// If we're a leaf, then our volume is just that of our object.
	if (IsLeaf())
	{
		if(std::is_class_v<BoundingSphere>)
			m_volume = T(m_rigidbody->GetBoundingSphere());
		else if(std::is_class_v<BoundingBox>)
			m_volume = T(m_rigidbody->GetBoundingBox());
	}
	// Otherwise we combine the volumes of our children.
	else
	{
		m_volume = T(children[0]->volume, children[1]->volume);
	}

	// If we have a parent, then tell our parent to recalculate its volume.
	if (parent && recurse)
	{
		parent->RecalculateBoundingVolume();
	}
}