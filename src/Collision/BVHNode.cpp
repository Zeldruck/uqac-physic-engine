#include <Collision/BVHNode.hpp>

template<class TBoundingVolume>
BVHNode<TBoundingVolume>::BVHNode(std::shared_ptr<Rigidbody> rigidbody, const TBoundingVolume& volume) :
	m_rigidbody(rigidbody),
	m_volume(volume),
	m_parent(nullptr)
{
}

template<class TBoundingVolume>
BVHNode<TBoundingVolume>::BVHNode(std::shared_ptr<BVHNode<TBoundingVolume>> node) :
	children(node->children),
	m_volume(node->volume),
	m_rigidbody(node->rigidbody),
	m_parent(node->parent)
{
}

template<class TBoundingVolume>
BVHNode<TBoundingVolume>::BVHNode(std::shared_ptr<BVHNode<TBoundingVolume>> node, std::shared_ptr<Rigidbody> body, const TBoundingVolume& volume) :
	children(node->children),
	m_volume(node->volume),
	m_rigidbody(node->rigidbody),
	m_parent(node->parent)
{
}

/* Delete this node, removing this first from the hierarchy, along with its associated rigidbody and child nodes. 
  This method deletes the node and all its children (but obviously not the rigidbodies).
  This also has the effect of deleting the siblings of this node and changing the parent node,
  so that it contains the data currently in one of the siblings.
  Finally, it forces the hierarchy above the current node to reconsider its bounding volume.
 */
template<class TBoundingVolume>
BVHNode<TBoundingVolume>::~BVHNode()
{
	// If we don't have a parent, then we ignore the sibling processing
	if (!m_parent)
		return;

	// Find our sibling
	std::shared_ptr<BVHNode<TBoundingVolume>> sibling;
	if (m_parent->children[0] == this)
		sibling = m_parent->children[1];
	else
		sibling = m_parent->children[0];

	// Write its data to our parent
	m_parent->m_volume = sibling->m_volume;
	m_parent->m_rigidbody = sibling->m_rigidbody;
	m_parent->children = sibling->children;

	// Delete the sibling (we blank its parent and children to avoid processing/deleting them)
	sibling->m_parent = nullptr;
	sibling->m_rigidbody = nullptr;
	sibling->children[0] = nullptr;
	sibling->children[1] = nullptr;
	sibing.reset();

	// Recalculate the parent's bounding volume
	m_parent->RecalculateBoundingVolume();

	// Delete our children (again, we remove their parent data so they are ignored by the destructor)
	if (children[0])
	{
		children[0]->m_parent = nullptr;
		children[0].reset();
	}

	if (children[1])
	{
		children[1]->m_parent = nullptr;
		children[1].reset();
	}
}

template<class TBoundingVolume>
bool BVHNode<TBoundingVolume>::IsLeaf() const
{
	return (m_rigidbody != nullptr);
}

template<class TBoundingVolume>
bool BVHNode<TBoundingVolume>::Overlaps(const std::shared_ptr<BVHNode<TBoundingVolume>> other) const
{
	return m_volume->Overlaps(other->volume);
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

template <class TBoundingVolume>
void BVHNode<TBoundingVolume>::Insert(std::shared_ptr<Rigidbody> newRigidbody, const TBoundingVolume& newVolume)
{
	// If we are a leaf, then the only option is to spawn two new children and place the new body in one.
	if (IsLeaf())
	{
		// Child one is a copy of us.
		children[0] = std::make_shared<BVHNode<TBoundingVolume>>(*this);

		// Child two holds the new body
		children[1] = std::make_shared<BVHNode<TBoundingVolume>>(*this, newRigidbody, newVolume);

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

template <class TBoundingVolume>
void BVHNode<TBoundingVolume>::RecalculateBoundingVolume(bool recurse /* = true */)
{
	// If we're a leaf, then our volume is just that of our object.
	if (IsLeaf())
	{
		m_volume = TBoundingVolume(m_rigidbody->GetBoundingVolume());
	}

	// Otherwise we combine the volumes of our children.
	else
	{
		m_volume = TBoundingVolume(children[0]->volume, children[1]->volume);
	}

	// If we have a parent, then tell our parent to recalculate its volume.
	if (parent && recurse)
	{
		parent->RecalculateBoundingVolume();
	}
}