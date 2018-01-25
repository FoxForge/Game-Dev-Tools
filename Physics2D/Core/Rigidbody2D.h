#ifndef _RIGIDBODY2D_H_
#define _RIGIDBODY2D_H_

// Get the physics utility delegates
#include "..\Utility\Physicsf.h"

// We need to include pooling to get inheritance properties
#include "..\Utility\Pooling.h"

// We need a vector2 for object sizes, to keep track of its own vector2
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace Physics2D
	{
		class RayCast2D;
		class RayCastResult2D;
		class PhysXAABB;
		class PhysXWorld2D;
		class Collider2D;
		class SeparatingAxis;

		enum BODY_TYPE : unsigned short;
		enum BODY_CONSTRAINT : unsigned short;


		class Rigidbody2D : public virtual StackPoolItem<Rigidbody2D>
		{
		private:
			const float ACC_MOD = 0.5F;

			int mPhaseID;
			float mAngle;
			float mInertia;
			float mInvInertia;
			float mMass;
			float mInvMass;
			float mTorqueCache;
			float mAngularVelocity;
			float mBiasRotation;
			Vector2 mPosition;
			Vector2 mForceCache;
			Vector2 mFacing;
			Vector2 mLinearVelocity;
			Vector2 mBiasVelocity;

			PhysXAABB* pAABB;
			PhysXWorld2D* pWorld;

			CollisionFilter2D mCollisionFilter;

			BODY_TYPE mBodyType;
			BODY_CONSTRAINT mBodyConstraint;

			Collider2D* pCollider;
			
			RayCast2D WorldToBodyRay(RayCast2D& ray);
			Vector2 BodyToWorldDirection(Vector2 direction);
			Vector2 WorldToBodyDirection(Vector2 direction);

			void UpdateAABB();
			void ClearForces();

			void Integrate();
			void IntegrateVelocity();
			void IntegrateMidPointAcceleration(Vector2 force, float torque);

			void SetStatic();
			void SetDynamic(float mass, BODY_CONSTRAINT constraint);

			void Init(
				Vector2 position,
				float radians,
				Collider2D* collider);
			
		public:
			Rigidbody2D();
			~Rigidbody2D();

			void InitDynamic(
				Vector2 position,
				float radians,
				float mass,
				Collider2D* collider,
				BODY_CONSTRAINT constraint);

			void InitStatic(
				Vector2 position,
				float radians,
				Collider2D* collider);

			void Set(Vector2 position, float radians);

			void Update();

			int GetPhaseID() const;
			void SetPhaseID(int id);

			Vector2 GetLinearVelocity() const;
			void SetLinearVelocity(Vector2 velocity);

			float GetAngularVelocity() const;
			void SetAngularVelocity(float torque);

			void AssignWorld(PhysXWorld2D* world);
			void SetCollisionFilter(CollisionFilter2D filter);
			void AddTorque(float torque);
			void AddForce(Vector2 force);
			void AddForceRelativeTo(Vector2 force, Vector2 point);
			void AddImpulse(Vector2 force, Vector2 direction);
			void AddBias(Vector2 force, Vector2 direction);

			bool CanCollide(Rigidbody2D* other) const;
			bool QueryPoint(Vector2 point);
			bool QueryCircle(Vector2 origin, float radius);
			bool RayCast(RayCast2D& ray, RayCastResult2D& result);
			bool CircleCast(
				RayCast2D& ray,
				float radius,
				RayCastResult2D& result);

			Vector2 WorldToBodyPoint(Vector2 point);
			Vector2 BodyToWorldPoint(Vector2 point);
			SeparatingAxis BodyToWorldAxis(SeparatingAxis axis);

			void FreeCollider();

			bool IsStatic() const;
			float GetMass() const;
			float GetAngle() const;
			float GetInvMass() const;
			float GetInertia() const;
			float GetInvInertia() const;
			float GetBiasRotation() const;

			Vector2 GetFacing() const;
			Vector2 GetPosition() const;
			Vector2 GetBiasVelocity() const;

			BODY_TYPE GetBodyType() const;

			PhysXAABB& GetAABB() const;
			PhysXWorld2D& GetWorld() const;
			
			Collider2D& GetCollider() const;

			static bool Filter(Rigidbody2D body, RigidbodyFilter2D filter);

			void Reset() override;
		};


		// POOLING

		class POOL_Rigidbody2D : public virtual StackPool<Rigidbody2D>
		{
		public:
			POOL_Rigidbody2D() { ; }
			Rigidbody2D* Allocate() override;
			void Deallocate(Rigidbody2D* obj) override;
		};
	}
}


#endif // !_RIGIDBODY2D_H_

