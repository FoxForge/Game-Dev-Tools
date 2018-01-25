#ifndef _RAYCASTRESULT2D_H_
#define _RAYCASTRESULT2D_H_

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

		class Collider2D;
		class Rigidbody2D;
		class RayCast2D;

		class RayCastResult2D
		{
		private:
			Collider2D* pCollider;
			float mDistance;
			Vector2* pNormal;

		public:
			RayCastResult2D();
			~RayCastResult2D();

			Rigidbody2D& GetBody() const;

			bool IsValid() const;
			bool IsContained() const;

			float GetDistance() const;
			void Set(Collider2D* collider, float distance, Vector2 normal);
			void SetNormal(Vector2 normal);
			void SetContained(Collider2D* collider);

			Vector2 GetNormal() const;
			Vector2 GetPoint(RayCast2D& ray) const;
		};
	}
}

#endif // !_RAYCASTRESULT2D_H_

