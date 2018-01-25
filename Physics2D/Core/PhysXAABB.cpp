#include "PhysXAABB.h"
#include "RayCast2D.h"

#include "..\..\Maths\Vector2.h"

#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		PhysXAABB::PhysXAABB(float top, float bottom, float left, float right)
		{
			mTop = top;
			mLeft = left;
			mRight = right;
			mBottom = bottom;
		}

		float PhysXAABB::GetWidth() const
		{
			return mRight - mLeft;
		}

		float PhysXAABB::GetHeight() const
		{
			return mTop - mBottom;
		}

		float PhysXAABB::GetPerimeter() const
		{
			return 2.0F * (GetWidth() + GetHeight());
		}

		bool PhysXAABB::RayCast(RayCast2D& ray) const
		{
			return RayCast(ray, mTop, mBottom, mLeft, mRight);
		}

		bool PhysXAABB::QueryPoint(Vector2 point) const
		{
			return
				mLeft <= point.GetX() &&
				mRight >= point.GetX() &&
				mBottom <= point.GetY() &&
				mTop >= point.GetY();
		}

		bool PhysXAABB::Intersect(PhysXAABB other) const
		{
			bool outside =
				mRight <= other.GetLeft() ||
				mLeft >= other.GetRight() ||
				mBottom >= other.GetTop() ||
				mTop <= other.GetBottom();

			return (outside == false);
		}

		bool PhysXAABB::CircleCastApprox(RayCast2D& ray, float radius) const
		{
			return RayCast(
				ray,
				mTop + radius,
				mBottom - radius,
				mLeft - radius,
				mRight + radius
			);
		}

		bool PhysXAABB::QueryCircleApprox(Vector2 origin, float radius) const
		{
			return
				(mLeft - radius) <= origin.GetX() ||
				(mRight + radius) >= origin.GetX() ||
				(mBottom - radius) <= origin.GetY() ||
				(mTop + radius) >= origin.GetY();
		}

		//static functions
		PhysXAABB PhysXAABB::CreateMerged(PhysXAABB aabb1, PhysXAABB aabb2)
		{		
			return PhysXAABB(
				std::max(aabb1.GetTop(), aabb2.GetTop()),
				std::min(aabb1.GetBottom(), aabb2.GetBottom()),
				std::min(aabb1.GetLeft(), aabb2.GetLeft()),
				std::max(aabb1.GetRight(), aabb2.GetRight())
			);
		}

		PhysXAABB PhysXAABB::CreateExpanded(PhysXAABB aabb, float expansionAmount)
		{
			return PhysXAABB(
				aabb.GetTop() + expansionAmount,
				aabb.GetBottom() - expansionAmount,
				aabb.GetLeft() - expansionAmount,
				aabb.GetRight() + expansionAmount
			);
		}

		bool PhysXAABB::RayCast(RayCast2D& ray, float top, float bottom, float left, float right)
		{
			float txmin = ((ray.IsNegativeDirectionX() ? right : left) - ray.GetOrigin().GetX()) * ray.GetInvDirection().GetX();
			float txmax = ((ray.IsNegativeDirectionX() ? left : right) - ray.GetOrigin().GetX()) * ray.GetInvDirection().GetX();

			float tymin = ((ray.IsNegativeDirectionX() ? top : bottom) - ray.GetOrigin().GetY()) * ray.GetInvDirection().GetY();
			float tymax = ((ray.IsNegativeDirectionX() ? bottom : top) - ray.GetOrigin().GetY()) * ray.GetInvDirection().GetY();

			if ((txmin > tymax) || (tymin > txmax))
				return false;

			if (tymax < txmax)
				txmax = tymax;

			return (txmax > 0.0f) && (txmin < ray.GetDistance());
		}
	}
}