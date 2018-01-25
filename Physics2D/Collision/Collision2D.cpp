#include "Collision2D.h"

#include "..\..\Maths\Vector2.h"
#include "..\Core\Collider2D.h"
#include "..\Core\PhysXWorld2D.h"
#include "..\Core\PolygonCollider2D.h"
#include "..\Core\RayCast2D.h"
#include "..\Core\RayCastResult2D.h"

#include "Manifold2D.h"
#include "SeparatingAxis.h"

#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		Manifold2D* Collision2D::Dispatch(
			PhysXWorld2D& world,
			Collider2D* a,
			Collider2D* b)
		{
			return CreatePolygonManifold2D(
				world,
				reinterpret_cast<PolygonCollider2D&>(*a),
				reinterpret_cast<PolygonCollider2D&>(*b));
		}

		// Need to return pointer to check against dispatched nullptrs
		Manifold2D* Collision2D::CreatePolygonManifold2D(
			PhysXWorld2D& world, 
			PolygonCollider2D& polyA, 
			PolygonCollider2D& polyB)
		{
			SeparatingAxis axis1, axis2;
			if (!GetMinSeparationAxis(polyA, polyB, axis1))
				return nullptr;

			if (!GetMinSeparationAxis(polyB, polyA, axis2))
				return nullptr;

			if (axis2.GetWidth() > axis1.GetWidth())
			{
				Manifold2D& manifold = world.AllocateManifold().Assign(world, &polyB, &polyA);
				AddContacts(polyB, polyA, axis2.GetNormal(), axis2.GetWidth(), manifold);
				return &manifold;
			}
			else
			{
				Manifold2D& manifold = world.AllocateManifold().Assign(world, &polyA, &polyB);
				AddContacts(polyA, polyB, axis1.GetNormal(), axis1.GetWidth(), manifold);
				return &manifold;
			}
		}

		int Collision2D::GetAxisPenetrationIndex(
			Vector2 origin,
			float radius,
			PolygonCollider2D& poly)
		{
			int foundIndex = 0;
			SeparatingAxis axis = poly.GetWorldAxes()[0];
			float dot = Vector2::DotProduct(axis.GetNormal(), origin);
			float maxDistance = dot - axis.GetWidth() - radius;
			if (maxDistance > 0.0f)
				return NULL_PHASE;

			for (int i = 1; i < poly.GetWorldCount(); i++)
			{
				axis = poly.GetWorldAxes()[i];
				dot = Vector2::DotProduct(axis.GetNormal(), origin);
				float distance = dot - axis.GetWidth() - radius;
				if (distance > 0.0f)
					return NULL_PHASE;

				if (distance > maxDistance)
				{
					maxDistance = distance;
					foundIndex = i;
				}
			}

			return foundIndex;
		}

		bool Collision2D::GetMinSeparationAxis(
			PolygonCollider2D& poly1,
			PolygonCollider2D& poly2,
			SeparatingAxis& outAxis)
		{
			Vector2 zero(0.0f, 0.0f);
			outAxis = SeparatingAxis(zero, FLT_MIN);
			for (int i = 0; i < poly1.GetWorldCount(); i++)
			{
				SeparatingAxis axis = poly1.GetWorldAxes()[i];
				float min = FLT_MAX;
				for (int j = 0; j < poly2.GetWorldCount(); j++)
				{
					Vector2 v = poly2.GetWorldVertices()[i];
					min = std::min(min, Vector2::DotProduct(axis.GetNormal(), v));
				}

				min -= axis.GetWidth();
				if (min > 0)
					return false;

				if (min > outAxis.GetWidth())
					outAxis = SeparatingAxis(axis.GetNormal(), min);
			}

			return true;
		}

		bool Collision2D::QueryPointInCircle(
			Vector2 point,
			Vector2 origin,
			float radius)
		{
			Vector2 delta = origin - point;
			return delta.MagnitudeSqrd() <= (radius * radius);
		}

		bool Collision2D::CircleRayCast(
			Collider2D* collider,
			Vector2 origin,
			float sqrRadius,
			RayCast2D& ray,
			RayCastResult2D& result)
		{
			Vector2 originDir = origin - ray.GetOrigin();
			if (originDir.MagnitudeSqrd() < sqrRadius)
			{
				result.SetContained(collider);
				return true;
			}

			float slope = Vector2::DotProduct(originDir, ray.GetDirection());
			if (slope < 0.0f)
				return false;

			float sqrSlope = slope * slope;
			float delta = sqrRadius + sqrSlope - Vector2::DotProduct(originDir, originDir);
			if (delta < 0.0f)
				return false;

			float distance = slope - sqrtf(delta);
			if (distance < 0.0f || distance > ray.GetDistance())
				return false;

			Vector2 normal = (ray.GetDirection() * distance - originDir).GetNormalized();
			result.Set(collider, distance, normal);
			return true;
		}

		void Collision2D::AddContacts(
			PolygonCollider2D& poly1,
			PolygonCollider2D& poly2,
			Vector2 normal,
			float penetration,
			Manifold2D& manifold)
		{
			// Attempt to Find and Add contact points
			if (poly1.FillContactManifold(poly2, normal, penetration, manifold, CONTACT_ACCURACY::FULL))
				return;

			if (poly2.FillContactManifold(poly1, normal, penetration, manifold, CONTACT_ACCURACY::FULL))
				return;

			// If some contacts points have been added then we can return
			if (manifold.GetContactCount() > 0)
				return;

			// Special cases may require slightly less accuracy
			if (poly1.FillContactManifold(poly2, normal, penetration, manifold, CONTACT_ACCURACY::PARTIAL))
				return;

			// Special cases may require to check the other side of the normal with less accuracy
			if (poly2.FillContactManifold(poly1, normal, penetration, manifold, CONTACT_ACCURACY::PARTIAL, true))
				return;
		}


	}
}