#ifndef _DYN2DTREEABB_H_
#define _DYN2DTREEABB_H_

#include "BroadPhase2D.h"
#include <functional>
#include <vector>

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		using Maths::Vector2;

		class PhysXAABB;
		class RayCast2D;

		class Dynamic2DTreeAABB : public virtual BroadPhase2D
		{
		private:

			enum NODE_TYPE_ID
			{
				LEFT = 0,
				RIGHT,
				PARENT
			};
			
			class AABBTreeNode
			{
			private:
				int mLeftID;
				int mRightID;
				int mParentID;
				int mTreeDepth;

				PhysXAABB* pAABB;
				Rigidbody2D* pBody;

			public:
				AABBTreeNode();
				AABBTreeNode(int parentID, int treeDepth);
				~AABBTreeNode();

				bool IsLeaf() const;
				int GetTreeDepth() const;
				int GetNodeID(NODE_TYPE_ID nodeType) const;
				
				PhysXAABB GetAABB() const;
				Rigidbody2D& GetRigidbody();

				void Initialize(int parentID, int treeDepth);
				void SetAABB(PhysXAABB aabb);
				void SetTreeDepth(int depth);
				void SetRigidBody(Rigidbody2D* body);
				void SetNodeID(NODE_TYPE_ID nodeType, int id);

				void Reset();
			};

			AABBTreeNode& GetNextNode();
			AABBTreeNode& AllocateTreeNode(int& outParentID);

			float GetVolumeAABB(int index, PhysXAABB& leafAABB);

			void FreeNode(int nodeID);
			void InsertLeaf(int leafID);
			void FixTreeTrail(int index);
			void RemoveLeaf(int leafID);
			void StartQuery(Buffer<Rigidbody2D*>& outbuffer);
			void ExpandChild(int query, Buffer<Rigidbody2D*>& outbuffer);
			void ExpandNode(AABBTreeNode node, Buffer<Rigidbody2D*>& outbuffer);
			void RotateByDepth(
				AABBTreeNode& P,
				AABBTreeNode& Q,
				AABBTreeNode& R,
				int primaryIndex,
				int rotationX,
				int rotationY,
				NODE_TYPE_ID rotationType
			);
			
			int GetBalanceIndex(int index);
			int FindBestChildOf(PhysXAABB& leafAABB);
			int GetRotationIndex(
				AABBTreeNode& P,
				AABBTreeNode& Q,
				AABBTreeNode& R,
				int primaryIndex,
				int rotationIndex,
				NODE_TYPE_ID rotationType
			);

			const int INIT_NODE_CAP = 16;
			std::vector<int> mQueryIndexVector;
			AABBTreeNode* pNodes;
			int mNodeCapacity;
			int mFreeIndex;
			int mNodeCount;
			int mRootID;

		public:
			Dynamic2DTreeAABB();
			~Dynamic2DTreeAABB();

			int GetCount() override;
			Rigidbody2D& operator [] (const int i) override;

			void AddBody(Rigidbody2D& body) override;
			void RemoveBody(Rigidbody2D& body) override;

			void QueryOverlap(PhysXAABB aabb, Buffer<Rigidbody2D*>& outbuffer);
			void QueryPoint(Vector2 point, Buffer<Rigidbody2D*>& outbuffer);
			void QueryCircle(Vector2 point, float radius, Buffer<Rigidbody2D*>& outbuffer);

			void QueryRayCast(RayCast2D& ray, Buffer<Rigidbody2D*>& outbuffer);
			void QueryCircleCast(RayCast2D& ray, float radius, Buffer<Rigidbody2D*>& outbuffer);
		};
	}
}

#endif // !_DYN2DTREEABB_H_

