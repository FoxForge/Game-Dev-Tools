#include "Manifold2D.h"

#include "ContactType.h"
#include "..\Core\PhysXWorld2D.h"
#include "..\Core\ContactPoint2D.h"
#include "..\Core\Collider2D.h"
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace Physics2D
	{
		Manifold2D::Manifold2D()
		{
			mUsedContactCount = 0;
			mFriction = 0.0f;
			mRestitution = 0.0f;
			pWorld = nullptr;
			pColliderA = nullptr;
			pColliderB = nullptr;
			mContactType = CONTACT_TYPE::PHYSICAL;
			Reset();
		}

		Manifold2D& Manifold2D::Assign(
			PhysXWorld2D& world,
			Collider2D* colliderA, 
			Collider2D* colldierB)
		{
			mUsedContactCount = 0;
			pWorld = &world;
			pColliderA = colliderA;
			pColliderB = colldierB;
			mContactType = (colliderA->IsTrigger() || colldierB->IsTrigger()) ? CONTACT_TYPE::TRIGGER : CONTACT_TYPE::PHYSICAL;
			mFriction = sqrtf(colliderA->GetFriction() * colldierB->GetFriction());
			mRestitution = sqrtf(colliderA->GetRestitution() * colldierB->GetRestitution());
			return *this;
		}

		Collider2D& Manifold2D::GetColliderA()
		{
			return *pColliderA;
		}

		Collider2D& Manifold2D::GetColliderB()
		{
			return *pColliderB;
		}

		int Manifold2D::GetContactCount() const
		{
			return mUsedContactCount;
		}

		float Manifold2D::GetRestitution() const
		{
			return mRestitution;
		}

		float Manifold2D::GetFriction() const
		{
			return mFriction;
		}

		CONTACT_TYPE Manifold2D::GetContactType() const
		{
			return mContactType;
		}

		bool Manifold2D::AddContact(
			Vector2 position, 
			Vector2 normal, 
			float penetration)
		{
			if (mUsedContactCount >= MAX_CONTACTS)
				return false;

			// Get a NEW contact point address from the world pooling system after assigning the values for it
			ptrContacts[mUsedContactCount++] = &pWorld->AllocateContact().Assign(position, normal, penetration);
			return true;
		}

		void Manifold2D::SetupCollision()
		{
			// Loop through all assigned contact points to setup variables for collision
			for (int i = 0; i < mUsedContactCount; i++)
				ptrContacts[i]->SetupCollision(*this);
		}

		void Manifold2D::SolveCollision(float elasticity)
		{
			// Loop through all assigned contact points and solve them with the given elastricity and this manifold
			for (int i = 0; i < mUsedContactCount; i++)
				ptrContacts[i]->SolveCollision(*this, elasticity);
		}

		void Manifold2D::ApplyCachedSolution()
		{
			// Loop through all assigned contact points and apply the cached solution from previous solving iterations
			for (int i = 0; i < mUsedContactCount; i++)
				ptrContacts[i]->ApplyCachedSolution(*this);
		}

		void Manifold2D::InvokeContactStay2D()
		{
			// If a contact function exists on A then invoke it passing B
			if (pColliderA->GetOnContactStay() != nullptr)
				pColliderA->GetOnContactStay().operator()(pColliderB->GetRigidBody());

			// If a contact function exists on B then invoke it passing A
			if (pColliderB->GetOnContactStay() != nullptr)
				pColliderB->GetOnContactStay().operator()(pColliderA->GetRigidBody());
		}

		void Manifold2D::InvokeContactEnter2D()
		{
			// If a contact function exists on A then invoke it passing B
			if (pColliderA->GetOnContactEnter() != nullptr)
				pColliderA->GetOnContactEnter().operator()(pColliderB->GetRigidBody());

			// If a contact function exists on B then invoke it passing A
			if (pColliderB->GetOnContactEnter() != nullptr)
				pColliderB->GetOnContactEnter().operator()(pColliderA->GetRigidBody());
		}

		void Manifold2D::InvokeContactExit2D()
		{
			// If a contact function exists on A then invoke it passing B
			if (pColliderA->GetOnContactExit() != nullptr)
				pColliderA->GetOnContactExit().operator()(pColliderB->GetRigidBody());

			// If a contact function exists on B then invoke it passing A
			if (pColliderB->GetOnContactExit() != nullptr)
				pColliderB->GetOnContactExit().operator()(pColliderA->GetRigidBody());
		}

		void Manifold2D::ClearContacts()
		{
			// Free all assigned contacts from their pool and reset the counter
			for (int i = 0; i < mUsedContactCount; i++)
				ptrContacts[i]->GetPool().Deallocate(ptrContacts[i]);

			mUsedContactCount = 0;
		}

		void Manifold2D::Reset()
		{
			// Used for pooling
			mFriction = 0.0f;
			mRestitution = 0.0f;
			pWorld = nullptr;
			pColliderA = nullptr;
			pColliderB = nullptr;
			mContactType = CONTACT_TYPE::PHYSICAL;
			ClearContacts();
		}

		//POOLING <--------------------------------------------

		Manifold2D* POOL_Manifold2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			Manifold2D* obj = mPoolStack.top();
			obj->Reset();
			obj->SetPool(this);
			mPoolStack.pop();
			return obj;
		}

		void POOL_Manifold2D::Deallocate(Manifold2D* obj)
		{
			obj->Reset();
			obj->SetPool(nullptr);
			mPoolStack.push(obj);
		}
	}
}