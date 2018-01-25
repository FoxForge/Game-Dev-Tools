#ifndef _COLLISION2D_H_
#define _COLLISION2D_H_

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		using Maths::Vector2;

		// Forward declarations
		class Collider2D;
		class PhysXWorld2D;
		class Manifold2D;
		class PolygonCollider2D;
		class SeparatingAxis;
		class RayCast2D;
		class RayCastResult2D;

		class Collision2D
		{
		private:
			static Manifold2D* CreatePolygonManifold2D(
				PhysXWorld2D& world,
				PolygonCollider2D& polyA,
				PolygonCollider2D& polyB);

			static bool GetMinSeparationAxis(
				PolygonCollider2D& poly1,
				PolygonCollider2D& poly2,
				SeparatingAxis& outAxis);

			static void AddContacts(
				PolygonCollider2D& poly1,
				PolygonCollider2D& poly2,
				Vector2 normal, 
				float penetration, 
				Manifold2D& manifold);

			Collision2D() { ; }

		public:
			static Manifold2D* Dispatch(
				PhysXWorld2D& world,
				Collider2D* a,
				Collider2D* b);

			static bool QueryPointInCircle(
				Vector2 point,
				Vector2 origin,
				float radius);

			static bool CircleRayCast(
				Collider2D* collider,
				Vector2 origin,
				float sqrRadius,
				RayCast2D& ray,
				RayCastResult2D& result);

			static int GetAxisPenetrationIndex(
				Vector2 origin,
				float radius, 
				PolygonCollider2D& poly);
		};
	}
}

#endif // !_COLLISION2D_H_

