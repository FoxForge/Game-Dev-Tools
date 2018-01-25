#ifndef _PHYSXWORLD2D_H_
#define _PHYSXWORLD2D_H_

// Get the physics utility, this is needed
#include "..\Utility\Physicsf.h"

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		template<class T> class Buffer;

		enum BODY_CONSTRAINT : unsigned short;

		class TriggerSolver2D;
		class CollisionSolver2D;
		class BroadPhase2D;
		class Dynamic2DTreeAABB;

		class Collider2D;
		class Rigidbody2D;
		class Manifold2D;
		class PolygonCollider2D;
		class ContactPoint2D;
		class RayCast2D;
		class RayCastResult2D;

		class POOL_Rigidbody2D;
		class POOL_Manifold2D;
		class POOL_PolygonCollider2D;
		class POOL_ContactPoint2D;

		class PhysXWorld2D
		{
		private:

			//World Properties
			float mDamping;
			float mFixedDeltaTime;

			//Manifold Solvers
			CollisionSolver2D* pPhysicalManifolds;
			TriggerSolver2D* pTriggerManifolds;

			//Phases
			BroadPhase2D* pDynamicBroadPhase;
			Dynamic2DTreeAABB* pStaticBroadPhase;

			//Buffers
			Buffer<Rigidbody2D*>* pPhaseBuffer;
			Buffer<Rigidbody2D*>* pQueryBuffer;
			Buffer<Rigidbody2D*>* pQueryOutput;

			//Component Pools
			POOL_Rigidbody2D* pBodyPool;
			POOL_Manifold2D* pManifoldPool;
			POOL_ContactPoint2D* pContactPool;
			POOL_PolygonCollider2D* pPolygonPool;

			void AddBodyToWorld(Rigidbody2D& body);
			void RemoveBodyFromWorld(Rigidbody2D& body);

			void BroadPhase();
			void QueryBufferNarrowPhase(Rigidbody2D& query);
			void NarrowPhase(Collider2D* a, Collider2D* b);

			void UpdateCollisions();

			void FreeManifolds();
			void FreeBody(Rigidbody2D& body);

		public:
			PhysXWorld2D(float worldDamping = DAMPING);
			~PhysXWorld2D();

			inline float GetWorldDamping() const { return mDamping; }
			inline float GetFixedDeltaTime() const { return mFixedDeltaTime; }

			PolygonCollider2D& CreatePolygonCollider2DWorldSpace(
				Vector2 worldVertices[],
				const int length,
				bool isTrigger,
				float friction = FRICTION,
				float restitution = RESTITUTION,
				OnContact2D enter = nullptr,
				OnContact2D stay = nullptr,
				OnContact2D exit = nullptr
			);

			Rigidbody2D& CreateStaticBody(
				Vector2 position,
				float radians,
				Collider2D* collider
			);

			Rigidbody2D& CreateDynamicBody(
				Vector2 position,
				float radians,
				float mass,
				Collider2D* collider,
				BODY_CONSTRAINT constraint
			);

			virtual void Update(); // < MAIN WORLD UPDATE >

			void DestroyBody(Rigidbody2D& body);

			Buffer<Rigidbody2D*> QueryPoint(Vector2 point, RigidbodyFilter2D filter = nullptr);

			Buffer<Rigidbody2D*> QueryCircle(
				Vector2 origin,
				float radius, 
				RigidbodyFilter2D filter = nullptr
			);

			bool RayCast(
				RayCast2D& ray,
				RayCastResult2D& result, 
				RigidbodyFilter2D filter = nullptr
			);

			bool CircleCast(
				RayCast2D& ray,
				float radius, 
				RayCastResult2D& result, 
				RigidbodyFilter2D filter = nullptr
			);

			ContactPoint2D& AllocateContact();
			Manifold2D& AllocateManifold();

			void FreeCollider(Collider2D* collider);
		};
	}
}

#endif // !_PHYSXWORLD2D_H_

