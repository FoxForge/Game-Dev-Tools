#ifndef _POLYGONCOL2D_H_
#define _POLYGONCOL2D_H_

// Include headers for inheritance
#include "Collider2D.h"
#include "..\Utility\Pooling.h"

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		//forward declarations
		using Maths::Vector2;

		class SeparatingAxis;
		class RayCast2D;
		class RayCastResult2D;
		class Rigidbody2D;
		class PhysXAABB;
		class Manifold2D;

		class PolygonCollider2D : public virtual Collider2D, public virtual StackPoolItem<PolygonCollider2D>
		{
		private:
			Vector2* pWorldVertices;
			SeparatingAxis* pWorldAxes;
			int mWorldCount;

			Vector2* pBodyVertices;
			SeparatingAxis* pBodyAxes;
			int mBodyCount;

			float CalculateInteria();

			void SetAttributeSizes(const int length);

			bool CircleCastEdges(
				RayCast2D& bodySpaceRay, 
				float radius,
				RayCastResult2D& result
			);

			bool CircleCastVertices(
				RayCast2D& bodySpaceRay,
				float radius,
				RayCastResult2D& result
			);

			//world to body
			void SetWorldToBodyVertices(
				Rigidbody2D* body,
				int count
			);

			static void CalculateAxes(
				Vector2* vertices,
				SeparatingAxis* memberAxisDestination,
				int commonLength
			);

			static PhysXAABB GetBoundsAABB(Vector2* vertices, int count);


		protected:

			//pure virtual overrides
			void ConfigureColldier() override;
			void ApplyBodyPosition() override;
			bool ColliderQueryPoint(Vector2 bodySpacePoint) override;
			bool ColliderQueryCircle(Vector2 bodySpaceOrigin, float radius) override;
			bool ColliderRayCast(RayCast2D& bodySpaceRay, RayCastResult2D& result) override;

			bool ColliderCircleCast(
				RayCast2D& bodySpaceRay,
				float radius,
				RayCastResult2D& result
			) override;

		public:
			PolygonCollider2D();
			~PolygonCollider2D();

			Vector2* GetWorldVertices() const;
			SeparatingAxis* GetWorldAxes() const;
			int GetWorldCount() const;

			void InitFromWorldVertices(
				Vector2 vertices[],
				const int length,
				bool isTrigger,
				float friction,
				float restitution,
				OnContact2D enter,
				OnContact2D stay,
				OnContact2D exit
			);
		
			bool ContainsPoint(Vector2 worldSpacePoint);

			bool ContainsPointPartial(
				Vector2 worldSpacePoint,
				Vector2 worldSpaceNormal,
				bool inverse
			);

			bool FillContactManifold(
				PolygonCollider2D other,
				Vector2 normal,
				float penetration,
				Manifold2D& manifold,
				CONTACT_ACCURACY accuracy,
				bool inverse = false
			);

			void Reset() override;
		};

		//POOLING

		class POOL_PolygonCollider2D : public virtual StackPool<PolygonCollider2D>
		{
		public:
			PolygonCollider2D* Allocate() override;
			void Deallocate(PolygonCollider2D* obj) override;
		}; 
	}
}


#endif // ! _POLYGONCOL2D_H_

