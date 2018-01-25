#ifndef _COLLIDER2D_H_
#define _COLLIDER2D_H_

// we need the physics utility delegates for OnContact invocations
#include "..\Utility\Physicsf.h"

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
		class Rigidbody2D;
		class RayCast2D;
		class RayCastResult2D;

		enum CONTACT_ACCURACY
		{
			FULL = 0,
			PARTIAL
		};

		class Collider2D
		{
		protected:
			OnContact2D mOnContactStay;
			OnContact2D mOnContactEnter;
			OnContact2D mOnContactExit;

			PhysXAABB* pWorldSpaceAABB;
			PhysXAABB* pBodySpaceAABB;
			Rigidbody2D* pBody;

			float mInertia;
			float mFriction;
			float mRestitution;
			bool mIsTrigger;

			void Initialize(
				bool isTrigger,
				float frction, 
				float restitution, 
				OnContact2D enter, 
				OnContact2D stay, 
				OnContact2D exit
			);

			virtual void ConfigureColldier() = 0;
			virtual void ApplyBodyPosition() = 0;
			virtual bool ColliderQueryPoint(Vector2 bodySpacePoint) = 0;
			virtual bool ColliderQueryCircle(Vector2 bodySpaceOrigin, float radius) = 0;
			virtual bool ColliderRayCast(RayCast2D& bodySpaceRay, RayCastResult2D& result) = 0;
			virtual bool ColliderCircleCast(
				RayCast2D& bodySpaceRay, 
				float radius, 
				RayCastResult2D& result
			) = 0;

		public:
			Collider2D();
			virtual ~Collider2D();

			bool IsTrigger() const;
			float GetFriction() const;
			float GetInertia() const;
			float GetRestitution() const;

			PhysXAABB GetWorldAABB() const;
			Rigidbody2D& GetRigidBody() const; // pointer

			OnContact2D GetOnContactStay() const;
			OnContact2D GetOnContactEnter() const;
			OnContact2D GetOnContactExit() const;

			void UpdatePosition();
			void AssignBody(Rigidbody2D* body); // body should be a pointer

			bool QueryPoint(Vector2 bodySpacePoint);
			bool QueryCircle(Vector2 bodySpaceOrigin, float radius);
			bool RayCast(RayCast2D& bodySpaceRay, RayCastResult2D& result);
			bool CircleCast(
				RayCast2D& bodySpaceRay,
				float radius, 
				RayCastResult2D& result
			);

			virtual void Reset();
		};
	}
}

#endif // !_COLLIDER2D_H_
