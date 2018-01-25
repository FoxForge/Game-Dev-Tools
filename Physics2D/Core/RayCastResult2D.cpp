#include "RayCastResult2D.h"

#include "..\..\Maths\Vector2.h"
#include "Collider2D.h"
#include "Rigidbody2D.h"
#include "RayCast2D.h"

namespace Fox
{
	namespace Physics2D
	{
		RayCastResult2D::RayCastResult2D()
		{
			pNormal = new Vector2();
		}

		RayCastResult2D::~RayCastResult2D()
		{
			delete pNormal;
		}

		Rigidbody2D& RayCastResult2D::GetBody() const
		{
			return pCollider->GetRigidBody();
		}

		bool RayCastResult2D::IsValid() const
		{
			return pCollider != nullptr;
		}

		bool RayCastResult2D::IsContained() const
		{
			return (IsValid() && mDistance == 0.0f);
		}

		float RayCastResult2D::GetDistance() const
		{
			return mDistance;
		}

		Vector2 RayCastResult2D::GetNormal() const
		{
			return *pNormal;
		}

		Vector2 RayCastResult2D::GetPoint(RayCast2D& ray) const
		{
			return (ray.GetOrigin() + (ray.GetDirection() * mDistance));
		}

		void RayCastResult2D::SetNormal(Vector2 normal)
		{
			*pNormal = normal;
		}

		void RayCastResult2D::Set(Collider2D* collider, float distance, Vector2 normal)
		{
			if (IsValid() && distance >= mDistance)
				return;

			pCollider = collider;
			mDistance = distance;
			*pNormal = normal;
		}

		void RayCastResult2D::SetContained(Collider2D* collider)
		{
			pCollider = collider;
			mDistance = 0.0f;
			pNormal->SetZero();
		}
	}
}