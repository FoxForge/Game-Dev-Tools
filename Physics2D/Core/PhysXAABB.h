#ifndef _PHYSXAABB_H_
#define _PHYSXAABB_H_

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		using Maths::Vector2;

		class RayCast2D;

		class PhysXAABB
		{
		private:
			float mTop;
			float mBottom;
			float mLeft;
			float mRight;

			static bool RayCast(RayCast2D& ray, float top, float bottom, float left, float right);

		public:
			PhysXAABB(float top, float bottom, float left, float right);
			~PhysXAABB() { ; }

			inline float GetTop() { return mTop; }
			inline float GetLeft() { return mLeft; }
			inline float GetRight() { return mRight; }
			inline float GetBottom() { return mBottom; }
			
			float GetWidth() const;
			float GetHeight() const;
			float GetPerimeter() const;

			bool RayCast(RayCast2D& ray) const;
			bool QueryPoint(Vector2 point) const;
			bool Intersect(PhysXAABB other) const;
			bool CircleCastApprox(RayCast2D& ray, float radius) const;
			bool QueryCircleApprox(Vector2 origin, float radius) const;

			static PhysXAABB CreateMerged(PhysXAABB aabb1, PhysXAABB aabb2);
			static PhysXAABB CreateExpanded(PhysXAABB aabb, float expansionAmount);
		};
	}
}

#endif // !_PHYSXAABB_H_

