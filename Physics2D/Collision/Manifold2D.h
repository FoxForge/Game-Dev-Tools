#ifndef _MANIFOLD2D_H_
#define _MANIFOLD2D_H_

// We need these includes for the constant values and pool inheritance
#include "..\Utility\Physicsf.h"
#include "..\Utility\Pooling.h"

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		enum CONTACT_TYPE : unsigned short;

		class ContactPoint2D;
		class PhysXWorld2D;
		class Collider2D;

		class Manifold2D : public virtual StackPoolItem<Manifold2D>
		{
		private:
			ContactPoint2D* ptrContacts[MAX_CONTACTS];

			CONTACT_TYPE mContactType;

			PhysXWorld2D* pWorld;
			Collider2D* pColliderA;
			Collider2D* pColliderB;

			int mUsedContactCount;
			float mRestitution;
			float mFriction;

			void ClearContacts();

		public:
			Manifold2D();
			~Manifold2D() { ; }

			Manifold2D& Assign(
				PhysXWorld2D& world, 
				Collider2D* colliderA, 
				Collider2D* colliderB);

			Collider2D& GetColliderA();
			Collider2D& GetColliderB();

			int GetContactCount() const;
			float GetRestitution() const;
			float GetFriction() const;
			CONTACT_TYPE GetContactType() const;

			bool AddContact(
				Vector2 position,
				Vector2 normal, 
				float penetration);

			void SetupCollision();
			void SolveCollision(float elasticity);
			void ApplyCachedSolution();
			void InvokeContactStay2D();
			void InvokeContactEnter2D();
			void InvokeContactExit2D();

			void Reset() override;
		};

		class POOL_Manifold2D : public virtual StackPool<Manifold2D>
		{
		public:
			Manifold2D* Allocate() override;
			void Deallocate(Manifold2D* obj) override;
		};
	}
}

#endif // !_MANIFOLD2D_H_

