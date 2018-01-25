#include "Dynamic2DTreeAABB.h"

#include "..\Utility\Buffer.h"
#include "..\Utility\Physicsf.h"
#include "..\Core\Rigidbody2D.h"
#include "..\Core\PhysXAABB.h"
#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		// NESTED Tree node Class <----------------------------------

		Dynamic2DTreeAABB::AABBTreeNode::AABBTreeNode()
		{
			mLeftID = 0;
			mRightID = 0;
			mParentID = 0;
			mTreeDepth = 0;

			pBody = nullptr;
			pAABB = new PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}

		Dynamic2DTreeAABB::AABBTreeNode::AABBTreeNode(int parentID, int treeDepth)
		{
			mParentID = parentID;
			mTreeDepth = treeDepth;

			pAABB = new PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}

		Dynamic2DTreeAABB::AABBTreeNode::~AABBTreeNode() 
		{ 
			delete pAABB;
		}

		bool Dynamic2DTreeAABB::AABBTreeNode::IsLeaf() const
		{
			return mLeftID == NULL_PHASE;
		}

		int Dynamic2DTreeAABB::AABBTreeNode::GetTreeDepth() const
		{
			return mTreeDepth;
		}

		int Dynamic2DTreeAABB::AABBTreeNode::GetNodeID(Dynamic2DTreeAABB::NODE_TYPE_ID nodeType) const
		{
			switch (nodeType)
			{
			case Dynamic2DTreeAABB::NODE_TYPE_ID::LEFT:
				return mLeftID;

			case Dynamic2DTreeAABB::NODE_TYPE_ID::RIGHT:
				return mRightID;

			case Dynamic2DTreeAABB::NODE_TYPE_ID::PARENT:
				return mParentID;

			default:
				return NULL_PHASE;
			}
		}

		PhysXAABB Dynamic2DTreeAABB::AABBTreeNode::GetAABB() const
		{
			return *pAABB;
		}

		Rigidbody2D& Dynamic2DTreeAABB::AABBTreeNode::GetRigidbody()
		{
			return *pBody;
		}

		void Dynamic2DTreeAABB::AABBTreeNode::Initialize(int parentID, int treeDepth)
		{
			mParentID = parentID;
			mTreeDepth = treeDepth;
		}

		void Dynamic2DTreeAABB::AABBTreeNode::SetTreeDepth(int depth)
		{
			mTreeDepth = depth;
		}

		void Dynamic2DTreeAABB::AABBTreeNode::SetAABB(PhysXAABB aabb)
		{
			*pAABB = aabb;
		}

		void Dynamic2DTreeAABB::AABBTreeNode::SetRigidBody(Rigidbody2D* body)
		{
			pBody = body;
		}

		void Dynamic2DTreeAABB::AABBTreeNode::SetNodeID(Dynamic2DTreeAABB::NODE_TYPE_ID nodeType, int id)
		{
			switch (nodeType)
			{
			case Dynamic2DTreeAABB::NODE_TYPE_ID::LEFT:
				mLeftID = id;
				break;

			case Dynamic2DTreeAABB::NODE_TYPE_ID::RIGHT:
				mRightID = id;
				break;

			case Dynamic2DTreeAABB::NODE_TYPE_ID::PARENT:
				mParentID = id;
				break;

			default:
				break;
			}
		}

		void Dynamic2DTreeAABB::AABBTreeNode::Reset()
		{
			mTreeDepth = 0;
			mLeftID = NULL_PHASE;
			mRightID = NULL_PHASE;
			mParentID = NULL_PHASE;

			pBody = nullptr;
			*pAABB = PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}

		// MAIN Dynamic Tree Phase Class <--------------------------------------

		Dynamic2DTreeAABB::Dynamic2DTreeAABB()
		{
			mFreeIndex = 0;
			mNodeCount = 0;
			mNodeCapacity = INIT_NODE_CAP;
			mRootID = NULL_PHASE;
			pNodes = new AABBTreeNode[INIT_NODE_CAP];

			for (int i = 0; i < mNodeCapacity - 1; i++)
				pNodes[i] = AABBTreeNode(i + 1, 1);

			pNodes[mNodeCapacity - 1] = AABBTreeNode(NULL_PHASE, 1);
		}

		Dynamic2DTreeAABB::~Dynamic2DTreeAABB()
		{
			delete[] pNodes;
		}

		int Dynamic2DTreeAABB::GetCount()
		{
			return mNodeCount;
		}

		Rigidbody2D& Dynamic2DTreeAABB::operator [] (const int i)
		{
			return pNodes[i].GetRigidbody();
		}
		
		float Dynamic2DTreeAABB::GetVolumeAABB(int index, PhysXAABB& leafAABB)
		{
			PhysXAABB aabb = PhysXAABB::CreateMerged(leafAABB, pNodes[index].GetAABB());
			if (pNodes[index].IsLeaf())
				return aabb.GetPerimeter();

			float oldArea = pNodes[index].GetAABB().GetPerimeter();
			float newArea = aabb.GetPerimeter();
			return (newArea - oldArea);
		}

		void Dynamic2DTreeAABB::FreeNode(int nodeID)
		{
			pNodes[nodeID].Initialize(mFreeIndex, NULL_PHASE);
			mFreeIndex = nodeID;
			mNodeCount--;
		}

		void Dynamic2DTreeAABB::InsertLeaf(int leafID)
		{
			if (mRootID == NULL_PHASE)
			{
				mRootID = leafID;
				pNodes[mRootID].SetNodeID(NODE_TYPE_ID::PARENT, NULL_PHASE);
				return;
			}

			// Find the best child of the leaf
			// We need to change this node later so keep a pointer to it
			AABBTreeNode& leafNode = pNodes[leafID];
			PhysXAABB leafAABB = leafNode.GetAABB();
			int childID = FindBestChildOf(leafAABB);

			// We also to change the child node later so keep a pointer to it
			AABBTreeNode& child = pNodes[childID];

			int newParentID = NULL_PHASE;
			int oldParentID = child.GetNodeID(NODE_TYPE_ID::PARENT);

			AABBTreeNode& newParent = AllocateTreeNode(newParentID);
			newParent.Initialize(oldParentID, child.GetTreeDepth() + 1);
			newParent.SetAABB(PhysXAABB::CreateMerged(leafAABB, child.GetAABB()));

			// Check the child against the tree
			if (oldParentID != NULL_PHASE)
			{
				// The child was not the root
				NODE_TYPE_ID parentType = (pNodes[oldParentID].GetNodeID(NODE_TYPE_ID::LEFT) == childID) ? NODE_TYPE_ID::LEFT : NODE_TYPE_ID::RIGHT;
				pNodes[oldParentID].SetNodeID(parentType, newParentID);
			}
			else
			{
				// The child was the root
				mRootID = newParentID; 
			}

			// Set split in tree
			newParent.SetNodeID(NODE_TYPE_ID::LEFT, childID);
			newParent.SetNodeID(NODE_TYPE_ID::RIGHT, leafID);

			// Set parents
			child.SetNodeID(NODE_TYPE_ID::PARENT, newParentID);
			leafNode.SetNodeID(NODE_TYPE_ID::PARENT, newParentID);

			// Walk back up the tree fixing depth and AABBs
			FixTreeTrail(newParentID);
		}

		void Dynamic2DTreeAABB::FixTreeTrail(int index)
		{
			// while we have not reached the root fix nodes from index trail
			while (index != NULL_PHASE)
			{
				// Get the index for the current node which needs fixing
				index = GetBalanceIndex(index);

				// Get the node to fix with it's relatives
				// we want to alter the node to fix, so keep a reference
				AABBTreeNode& nodeToFix = pNodes[index];
				AABBTreeNode left = pNodes[nodeToFix.GetNodeID(NODE_TYPE_ID::LEFT)];
				AABBTreeNode right = pNodes[nodeToFix.GetNodeID(NODE_TYPE_ID::RIGHT)];

				// Fix the node AABB and depth
				nodeToFix.SetAABB(PhysXAABB::CreateMerged(left.GetAABB(), right.GetAABB()));
				nodeToFix.SetTreeDepth(1 + std::max(left.GetTreeDepth(), right.GetTreeDepth()));

				// Move through the tree via parents
				index = nodeToFix.GetNodeID(NODE_TYPE_ID::PARENT);
			}
		}

		void Dynamic2DTreeAABB::RemoveLeaf(int leafID)
		{
			// If we have reached the root then we can set it to null phase index and return out
			if (leafID == mRootID)
			{
				mRootID = NULL_PHASE;
				return;
			}

			// find the parent of the ID given
			int parentID = pNodes[leafID].GetNodeID(NODE_TYPE_ID::PARENT);
			AABBTreeNode parent = pNodes[parentID];

			// find the child of the ID given (we want to alter it later so keep a reference)
			int childID = (parent.GetNodeID(NODE_TYPE_ID::LEFT) == leafID) ? parent.GetNodeID(NODE_TYPE_ID::RIGHT) : leafID;
			AABBTreeNode& child = pNodes[childID];

			// Check parent of this parent ID against the root
			int grandParentID = parent.GetNodeID(NODE_TYPE_ID::PARENT);
			if (grandParentID != NULL_PHASE)
			{
				// Connect the grandparents to children where this parent node will be removed
				NODE_TYPE_ID grandParentType = (pNodes[grandParentID].GetNodeID(NODE_TYPE_ID::LEFT) == parentID) ? NODE_TYPE_ID::LEFT : NODE_TYPE_ID::RIGHT;
				pNodes[grandParentID].SetNodeID(grandParentType, childID);
				child.SetNodeID(NODE_TYPE_ID::PARENT, grandParentID);
				FreeNode(parentID);
				FixTreeTrail(grandParentID);
			}
			else
			{
				// Grandparent will be root so we can skip a few calculations
				mRootID = childID;
				child.SetNodeID(NODE_TYPE_ID::PARENT, NULL_PHASE);
				FreeNode(parentID);
			}
		}

		void Dynamic2DTreeAABB::StartQuery(Buffer<Rigidbody2D*>& outbuffer)
		{
			// Clear the queries and start expansion of the tree from the root
			mQueryIndexVector.clear();
			ExpandChild(mRootID, outbuffer);
		}

		void Dynamic2DTreeAABB::ExpandNode(Dynamic2DTreeAABB::AABBTreeNode node, Buffer<Rigidbody2D*>& outbuffer)
		{
			// Expand the children of a node - continue to pass buffer so items can attach themselves
			ExpandChild(node.GetNodeID(NODE_TYPE_ID::LEFT), outbuffer);
			ExpandChild(node.GetNodeID(NODE_TYPE_ID::RIGHT), outbuffer);
		}

		void Dynamic2DTreeAABB::ExpandChild(int query, Buffer<Rigidbody2D*>& outbuffer)
		{
			// Have we reached a NULL part of the tree?
			if (query == NULL_PHASE)
				return;

			// Have we reached the end branches of the tree?
			AABBTreeNode node = pNodes[query];
			if (node.IsLeaf())
			{
				// Yes? then we have checked through and can be accepted to the buffer
				outbuffer.Add(&node.GetRigidbody()); // <-- get the address of the referenced rigidbody
			}
			else
			{
				// No? push the node index so it can be checked
				mQueryIndexVector.push_back(query);
			}
		}

		void Dynamic2DTreeAABB::RotateByDepth(
			Dynamic2DTreeAABB::AABBTreeNode& P,
			Dynamic2DTreeAABB::AABBTreeNode& Q,
			Dynamic2DTreeAABB::AABBTreeNode& R,
			int primaryIndex,
			int rotationX,
			int rotationY,
			NODE_TYPE_ID rotationType)
		{
			// Apply matrix rotation to the nodes for their indexed connections
			AABBTreeNode X = pNodes[rotationX];
			AABBTreeNode& Y = pNodes[rotationY];

			// rotate indices
			P.SetNodeID(rotationType, rotationY);
			R.SetNodeID(NODE_TYPE_ID::RIGHT, rotationX);
			Y.SetNodeID(NODE_TYPE_ID::PARENT, primaryIndex);

			// Every change in rotation will mean a fix for the AABB and tree depth
			P.SetAABB(PhysXAABB::CreateMerged(Q.GetAABB(), Y.GetAABB()));
			R.SetAABB(PhysXAABB::CreateMerged(P.GetAABB(), X.GetAABB()));
			P.SetTreeDepth(1 + std::max(Q.GetTreeDepth(), Y.GetTreeDepth()));
			R.SetTreeDepth(1 + std::max(P.GetTreeDepth(), X.GetTreeDepth()));
		}


		int Dynamic2DTreeAABB::GetBalanceIndex(int index)
		{
			// Obtain a balance for tree depth
			AABBTreeNode& A = pNodes[index];
			if (A.IsLeaf() || A.GetTreeDepth() < 2)
				return index;

			int leftID = A.GetNodeID(NODE_TYPE_ID::LEFT);
			int rightID = A.GetNodeID(NODE_TYPE_ID::RIGHT);

			AABBTreeNode& B = pNodes[leftID];
			AABBTreeNode& C = pNodes[rightID];
			int balance = C.GetTreeDepth() - B.GetTreeDepth();

			// B is at a notable greater depth
			// Rotate C up
			if (balance > 1)
				return GetRotationIndex(A, B, C, index, rightID, NODE_TYPE_ID::RIGHT);

			// C is at a notable greater depth
			// Rotate B up
			if (balance < -1)
				return GetRotationIndex(A, C, B, index, leftID, NODE_TYPE_ID::LEFT);

			// Left and Right are at roughly same depth
			return index;
		}

		int Dynamic2DTreeAABB::FindBestChildOf(PhysXAABB& leafAABB)
		{
			int index = mRootID;
			while (!pNodes[index].IsLeaf())
			{
				AABBTreeNode indexNode = pNodes[index];
				int child1 = indexNode.GetNodeID(NODE_TYPE_ID::LEFT);
				int child2 = indexNode.GetNodeID(NODE_TYPE_ID::RIGHT);
				float area = indexNode.GetAABB().GetPerimeter();

				// Cost of creating a new parent for this node and the new leaf
				PhysXAABB combinedAABB = PhysXAABB::CreateMerged(indexNode.GetAABB(), leafAABB);
				float combinedArea = combinedAABB.GetPerimeter();

				// Minimum cost of pushing the leaf further down the tree
				float cost = 2.0f * combinedArea;
				float inheritCost = 2.0f * (combinedArea - area);
				float cost1 = GetVolumeAABB(child1, leafAABB) + inheritCost;
				float cost2 = GetVolumeAABB(child2, leafAABB) + inheritCost;

				// Descend according to the minimum cost.
				if ((cost < cost1) && (cost1 < cost2))
					break;

				// Descend
				index = (cost1 < cost2) ? child1 : child2;
			}

			// Once we have descended to find a solution, return the index for node
			return index;
		}

		int Dynamic2DTreeAABB::GetRotationIndex(
			Dynamic2DTreeAABB::AABBTreeNode& P,
			Dynamic2DTreeAABB::AABBTreeNode& Q,
			Dynamic2DTreeAABB::AABBTreeNode& R,
			int primaryIndex,
			int rotationIndex,
			NODE_TYPE_ID rotationType)
		{
			// use index as side of rotation
			int rotationX = R.GetNodeID(NODE_TYPE_ID::LEFT);
			int rotationY = R.GetNodeID(NODE_TYPE_ID::RIGHT);

			// Rotate P and R connections
			R.SetNodeID(NODE_TYPE_ID::LEFT, primaryIndex);
			R.SetNodeID(NODE_TYPE_ID::PARENT, P.GetNodeID(NODE_TYPE_ID::PARENT));
			P.SetNodeID(NODE_TYPE_ID::PARENT, rotationIndex);

			// P's old parent should point to R
			if (R.GetNodeID(NODE_TYPE_ID::PARENT) == NULL_PHASE)
			{
				mRootID = rotationIndex;
			}
			else
			{
				NODE_TYPE_ID parentType = (pNodes[R.GetNodeID(NODE_TYPE_ID::PARENT)].GetNodeID(
					NODE_TYPE_ID::LEFT) == primaryIndex) ? NODE_TYPE_ID::LEFT : NODE_TYPE_ID::RIGHT;

				pNodes[R.GetNodeID(NODE_TYPE_ID::PARENT)].SetNodeID(parentType, rotationIndex);
			}

			// Rotate based on depth within the tree
			bool largerDepthX = pNodes[rotationX].GetTreeDepth() > pNodes[rotationY].GetTreeDepth();
			RotateByDepth(
				P,
				Q,
				R,
				primaryIndex,
				(largerDepthX) ? rotationX : rotationY,
				(largerDepthX) ? rotationY : rotationX,
				rotationType);

			return rotationIndex;
		}

		Dynamic2DTreeAABB::AABBTreeNode& Dynamic2DTreeAABB::GetNextNode()
		{
			int index = mQueryIndexVector.back();
			mQueryIndexVector.pop_back();
			return pNodes[index];
		}

		Dynamic2DTreeAABB::AABBTreeNode& Dynamic2DTreeAABB::AllocateTreeNode(int& outParentID)
		{
			// If we have no more free indices in the tree then we need to expand capacity
			// Default behaviour to double in size
			if (mFreeIndex == NULL_PHASE)
			{
				// Create new array for doubled size of current node storage
				AABBTreeNode* newNodes = new AABBTreeNode[2 * mNodeCapacity];

				// Copy all existing node data accross to new array
				for (int i = 0; i < mNodeCapacity; i++)
					newNodes[i] = pNodes[i];

				// Set the doubled capacity
				mNodeCapacity *= 2;

				// For all the new nodes, initialize them with base tree values 
				for (int i = mNodeCount; i < mNodeCapacity - 1; i++)
					newNodes[i] = AABBTreeNode(i + 1, -1);

				// Set the tail of the array
				newNodes[mNodeCapacity - 1] = AABBTreeNode(NULL_PHASE, -1);

				// Update the free index
				mFreeIndex = mNodeCount;

				// Pointer management
				delete[] pNodes;
				pNodes = newNodes;
			}

			// Give out the free index for a parent
			outParentID = mFreeIndex;

			// Set the next free index
			mFreeIndex = pNodes[outParentID].GetNodeID(NODE_TYPE_ID::PARENT);

			// Reset the node to be given back
			pNodes[outParentID].Reset();

			// Increase the node count and return the node reference
			mNodeCount++;
			return pNodes[outParentID];
		}

		void Dynamic2DTreeAABB::AddBody(Rigidbody2D& body)
		{
			// Get a new node from the tree and corresponsding ID
			int phaseID = NULL_PHASE;
			AABBTreeNode& node = AllocateTreeNode(phaseID);

			// Set the node properties based from rigidbody AABB
			// Expanded by default amount
			node.SetAABB(PhysXAABB::CreateExpanded(body.GetAABB(), AABB_EXTENSION));
			node.SetRigidBody(&body);
			node.SetTreeDepth(0);

			// Insert into the tree
			InsertLeaf(phaseID);

			// Given the phase ID to the rigidbody
			body.SetPhaseID(phaseID);
		}

		void Dynamic2DTreeAABB::RemoveBody(Rigidbody2D& body)
		{
			// Get the phase ID from the rigid body and remove corresponding node from the tree
			int phaseID = body.GetPhaseID();
			RemoveLeaf(phaseID);

			// Free the node so it can be given back out when requested
			FreeNode(phaseID);

			// Set the body phaseID to invalid
			body.SetPhaseID(NULL_PHASE);
		}

		void Dynamic2DTreeAABB::QueryOverlap(PhysXAABB aabb, Buffer<Rigidbody2D*>& outbuffer)
		{
			StartQuery(outbuffer);
			while (mQueryIndexVector.size() > 0)
			{
				AABBTreeNode node = GetNextNode();
				if (node.GetAABB().Intersect(aabb))
					ExpandNode(node, outbuffer);
			}
		}

		void Dynamic2DTreeAABB::QueryPoint(Vector2 point, Buffer<Rigidbody2D*>& outbuffer)
		{
			StartQuery(outbuffer);
			while (mQueryIndexVector.size() > 0)
			{
				AABBTreeNode node = GetNextNode();
				if (node.GetAABB().QueryPoint(point))
					ExpandNode(node, outbuffer);
			}
		}

		void Dynamic2DTreeAABB::QueryCircle(Vector2 point, float radius, Buffer<Rigidbody2D*>& outbuffer)
		{
			StartQuery(outbuffer);
			while (mQueryIndexVector.size() > 0)
			{
				AABBTreeNode node = GetNextNode();
				if (node.GetAABB().QueryCircleApprox(point, radius))
					ExpandNode(node, outbuffer);
			}
		}

		void Dynamic2DTreeAABB::QueryRayCast(RayCast2D& ray, Buffer<Rigidbody2D*>& outbuffer)
		{
			StartQuery(outbuffer);
			while (mQueryIndexVector.size() > 0)
			{
				AABBTreeNode node = GetNextNode();
				if (node.GetAABB().RayCast(ray))
					ExpandNode(node, outbuffer);
			}
		}

		void Dynamic2DTreeAABB::QueryCircleCast(RayCast2D& ray, float radius, Buffer<Rigidbody2D*>& outbuffer)
		{
			StartQuery(outbuffer);
			while (mQueryIndexVector.size() > 0)
			{
				AABBTreeNode node = GetNextNode();
				if (node.GetAABB().CircleCastApprox(ray, radius))
					ExpandNode(node, outbuffer);
			}
		}
	}
}