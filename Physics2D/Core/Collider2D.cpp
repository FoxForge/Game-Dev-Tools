#include "Collider2D.h"
#include "Rigidbody2D.h"
#include "PhysXAABB.h"
#include "RayCast2D.h"
#include "RayCastResult2D.h"

namespace Fox
{
	namespace Physics2D
	{
		Collider2D::Collider2D()
		{
			pWorldSpaceAABB = new PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
			pBodySpaceAABB = new PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}

		Collider2D::~Collider2D()
		{
			delete pWorldSpaceAABB;
			delete pBodySpaceAABB;
		}

		bool Collider2D::IsTrigger() const
		{
			return mIsTrigger;
		}

		float Collider2D::GetFriction() const
		{
			return mFriction;
		}

		float Collider2D::GetInertia() const
		{
			return mInertia;
		}

		float Collider2D::GetRestitution() const
		{
			return mRestitution;
		}

		OnContact2D Collider2D::GetOnContactEnter() const
		{
			return mOnContactEnter;
		}

		OnContact2D Collider2D::GetOnContactStay() const
		{
			return mOnContactStay;
		}

		OnContact2D Collider2D::GetOnContactExit() const
		{
			return mOnContactExit;
		}

		PhysXAABB Collider2D::GetWorldAABB() const
		{
			return *pWorldSpaceAABB;
		}

		Rigidbody2D& Collider2D::GetRigidBody() const
		{
			return *pBody;
		}

		void Collider2D::AssignBody(Rigidbody2D* body)
		{
			pBody = body;
			ConfigureColldier();
		}

		void Collider2D::UpdatePosition()
		{
			ApplyBodyPosition();
		}

		void Collider2D::Initialize(
			bool isTrigger,
			float friction,
			float restitution,
			OnContact2D enter,
			OnContact2D stay,
			OnContact2D exit)
		{
			mIsTrigger = isTrigger;
			mFriction = friction;
			mRestitution = restitution;
			mOnContactEnter = enter;
			mOnContactStay = stay;
			mOnContactExit = exit;
		}

		bool Collider2D::QueryPoint(Vector2 bodySpacePoint)
		{
			if (pBodySpaceAABB->QueryPoint(bodySpacePoint))
				return ColliderQueryPoint(bodySpacePoint);

			return false;
		}

		bool Collider2D::QueryCircle(Vector2 bodySpaceOrigin, float radius)
		{
			if (pBodySpaceAABB->QueryCircleApprox(bodySpaceOrigin, radius))
				return ColliderQueryCircle(bodySpaceOrigin, radius);

			return false;
		}

		bool Collider2D::RayCast(RayCast2D& bodySpaceRay, RayCastResult2D& result)
		{
			if (pBodySpaceAABB->RayCast(bodySpaceRay))
				return ColliderRayCast(bodySpaceRay, result);

			return false;
		}

		bool Collider2D::CircleCast(
			RayCast2D& bodySpaceRay,
			float radius,
			RayCastResult2D& result)
		{
			if (pBodySpaceAABB->CircleCastApprox(bodySpaceRay, radius))
				return ColliderCircleCast(bodySpaceRay, radius, result);

			return false;
		}

		void Collider2D::Reset()
		{
			mOnContactEnter = nullptr;
			mOnContactStay = nullptr;
			mOnContactExit = nullptr;
			pBody = nullptr;
			mFriction = 0.0f;
			mRestitution = 0.0f;
			mInertia = 0.0f;
			*pBodySpaceAABB = PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
			*pWorldSpaceAABB = PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}
	}
}