#ifndef _CONTACTPOINT2D_H_
#define _CONTACTPOINT2D_H_

// We have to include vector2 as we need to know the size
#include "..\..\Maths\Vector2.h"

// Include pooling to get inheritance properties
#include "..\Utility\Pooling.h"

namespace Fox
{
	namespace Physics2D
	{
		using Maths::Vector2;

		class Rigidbody2D;
		class Manifold2D;

		class ContactPoint2D : public virtual StackPoolItem<ContactPoint2D>
		{
		private:
			Vector2 mPosition;
			Vector2 mToALeft;
			Vector2 mToBLeft;
			Vector2 mNormal;
			Vector2 mToA;
			Vector2 mToB;
			float mBias;
			float mRestitution;
			float mPenetration;
			float mNormalMassRatio;
			float mTangentMassRatio;
			float mCachedBiasImpulse;
			float mCachedNormalImpulse;
			float mCachedTangentImpulse;

			float GetBiasDistance(float distance);
			float ContactScalar(Rigidbody2D& a, Rigidbody2D& b, Vector2 normal);
			Vector2 GetRelativeVelocity(Rigidbody2D& a, Rigidbody2D& b);
			void ApplyNormalBiasImpulse(Rigidbody2D& a, Rigidbody2D& b, float normalBiasImpulse);

			void ApplyContactImpulse(
				Rigidbody2D& a,
				Rigidbody2D& b,
				float normalImpulseMagnitude,
				float tangentImpulseMagnitude
			);

		public:
			ContactPoint2D();
			~ContactPoint2D() { ; }

			ContactPoint2D& Assign(Vector2 position, Vector2 normal, float penetration);
			void SetupCollision(Manifold2D& manifold);
			void SolveCollision(Manifold2D& manifold, float elasticity);
			void ApplyCachedSolution(Manifold2D& manifold);

			void Reset() override;
		};


		//POOLING

		class POOL_ContactPoint2D : public virtual StackPool<ContactPoint2D>
		{
		public:
			ContactPoint2D* Allocate() override;
			void Deallocate(ContactPoint2D* obj) override;
		};
	}
}

#endif // !_CONTACTPOINT2D_H_
