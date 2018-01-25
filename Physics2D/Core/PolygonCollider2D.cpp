#include "PolygonCollider2D.h"

#include "..\..\Maths\Vector2.h"
#include "..\Utility\Physicsf.h"
#include "..\Collision\SeparatingAxis.h"
#include "..\Collision\Manifold2D.h"
#include "..\Collision\ContactType.h"
#include "..\Collision\Collision2D.h"

#include "PhysXAABB.h"
#include "RayCast2D.h"
#include "RayCastResult2D.h"
#include "Rigidbody2D.h"

#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		PolygonCollider2D::PolygonCollider2D()
		{
			Reset();
		}

		PolygonCollider2D::~PolygonCollider2D()
		{
			if (pWorldVertices)
				delete[] pWorldVertices;

			if (pBodyVertices)
				delete[] pBodyVertices;

			if (pWorldAxes)
				delete[] pWorldAxes;

			if (pBodyAxes)
				delete[] pBodyAxes;
		}

		int PolygonCollider2D::GetWorldCount() const
		{
			return mWorldCount;
		}

		Vector2* PolygonCollider2D::GetWorldVertices() const
		{
			return pWorldVertices;
		}

		SeparatingAxis* PolygonCollider2D::GetWorldAxes() const
		{
			return pWorldAxes;
		}

		void PolygonCollider2D::InitFromWorldVertices(
			Vector2 vertices[],
			const int length,
			bool isTrigger,
			float friction,
			float restitution,
			OnContact2D enter,
			OnContact2D stay,
			OnContact2D exit)
		{
			Initialize(
				isTrigger,
				friction, 
				restitution, 
				enter, 
				stay, 
				exit);

			mBodyCount = 0;
			mWorldCount = length;
			SetAttributeSizes(length);

			for (int i = 0; i < length; i++)
				pWorldVertices[i] = vertices[i];

			CalculateAxes(vertices, pWorldAxes, length);
			*pWorldSpaceAABB = GetBoundsAABB(vertices, length);
		}

		void PolygonCollider2D::CalculateAxes(Vector2* vertices, SeparatingAxis* memberAxisDestination, int commonLength)
		{
			for (int i = 0; i < commonLength; i++)
			{
				Vector2 u = vertices[i];
				Vector2 v = vertices[(i + 1) % commonLength];
				Vector2 normal = (v - u).RotateLeft().GetNormalized();
				memberAxisDestination[i] = SeparatingAxis(normal, Vector2::DotProduct(normal, u));
			}
		}

		PhysXAABB PolygonCollider2D::GetBoundsAABB(Vector2* vertices, int count)
		{
			float top = vertices[0].GetY();
			float bottom = vertices[0].GetY();
			float left = vertices[0].GetX();
			float right = vertices[0].GetX();

			for (int i = 1; i < count; i++)
			{
				//max
				top = std::max(top, vertices[i].GetY());
				right = std::max(right, vertices[i].GetX());

				//min
				bottom = std::min(bottom, vertices[i].GetY());
				left = std::min(left, vertices[i].GetX());
			}

			return PhysXAABB(top, bottom, left, right);
		}

		bool PolygonCollider2D::ContainsPoint(Vector2 worldSpacePoint)
		{
			for (int i = 0; i < mWorldCount; i++)
			{
				if (Vector2::DotProduct(pWorldAxes[i].GetNormal(), worldSpacePoint) > pWorldAxes[i].GetWidth())
				{
					return false;
				}
			}

			return true;
		}

		bool PolygonCollider2D::ContainsPointPartial(Vector2 worldSpacePoint, Vector2 worldSpaceNormal, bool inverse)
		{
			if (inverse)
				worldSpaceNormal.SetVector(-worldSpaceNormal[0], -worldSpaceNormal[1]);

			for (int i = 0; i < mWorldCount; i++)
			{
				if (Vector2::DotProduct(pWorldAxes[i].GetNormal(), worldSpaceNormal) >= 0.0f &&
					Vector2::DotProduct(pWorldAxes[i].GetNormal(), worldSpacePoint) > pWorldAxes[i].GetWidth())
				{
					return false;
				}
			}

			return true;
		}

		bool PolygonCollider2D::FillContactManifold(
			PolygonCollider2D other, 
			Vector2 normal,
			float penetration,
			Manifold2D& manifold,
			CONTACT_ACCURACY accuracy, 
			bool inverse)
		{
			for (int i = 0; i < mWorldCount; i++)
			{
				// Search vertices related to contact accuracy
				if (accuracy == CONTACT_ACCURACY::FULL ? !other.ContainsPoint(pWorldVertices[i]) : !other.ContainsPointPartial(pWorldVertices[i], normal, inverse))
					continue;

				// If the vertex can be added, attempt to add to the manifold
				// If we cannot add anymore contacts then we have filled the manifold - return true
				if (!manifold.AddContact(pWorldVertices[i], normal, penetration))
					return true;
			}

			return false;
		}


		float PolygonCollider2D::CalculateInteria()
		{
			float sum1 = 0.0f;
			float sum2 = 0.0f;

			for (int i = 0; i < mBodyCount; i++)
			{
				Vector2 u = pBodyVertices[i];
				Vector2 v = pBodyVertices[(i + 1) % mBodyCount];
				float a = Vector2::CrossProduct(v, u);
				float b = u.MagnitudeSqrd() + v.MagnitudeSqrd() + Vector2::DotProduct(u, v);
				sum1 += a * b;
				sum2 += a;
			}

			return (sum1 / (6.0f * sum2));
		}

		void PolygonCollider2D::SetAttributeSizes(const int length)
		{
			if (pWorldVertices)
				delete[] pWorldVertices;

			if (pBodyVertices)
				delete[] pBodyVertices;

			if (pWorldAxes)
				delete[] pWorldAxes;

			if (pBodyAxes)
				delete[] pBodyAxes;

			pWorldVertices = new Vector2[length];
			pBodyVertices = new Vector2[length];
			
			pWorldAxes = new SeparatingAxis[length];
			pBodyAxes = new SeparatingAxis[length];
		}

		void PolygonCollider2D::SetWorldToBodyVertices(Rigidbody2D* body, int count)
		{
			for (int i = 0; i < count; i++)
			{
				pBodyVertices[i] = body->WorldToBodyPoint(pWorldVertices[i]);
			}
		}

		bool PolygonCollider2D::CircleCastEdges(
			RayCast2D& bodySpaceRay,
			float radius,
			RayCastResult2D& result)
		{
			int foundIndex = NULL_PHASE;
			bool canContain = true;
			float shortestDistance = FLT_MAX;
			Vector2 u = bodySpaceRay.GetDirection().RotateLeft();

			for (int i = 0; i < mBodyCount; i++)
			{
				SeparatingAxis axis = pBodyAxes[i];
				Vector2 extension = axis.GetNormal() * radius;
				Vector2 a = pBodyVertices[i] + extension;
				Vector2 b = pBodyVertices[(i + 1) % mBodyCount] + extension;

				if (canContain)
				{
					float projectedMagnitude = Vector2::DotProduct(axis.GetNormal(), bodySpaceRay.GetOrigin()) - axis.GetWidth();
					if (projectedMagnitude > radius)
					{
						canContain = false;
					}
					else if (projectedMagnitude > 0.0f)
					{
						float delta = Vector2::CrossProduct(axis.GetNormal(), bodySpaceRay.GetOrigin());
						if (delta > Vector2::CrossProduct(axis.GetNormal(), a) || delta < Vector2::CrossProduct(axis.GetNormal(), b))
						{
							canContain = false;
						}
					}
				}

				if (Vector2::DotProduct(axis.GetNormal(), bodySpaceRay.GetDirection()) >= 0.0f)
					continue;

				Vector2 v = bodySpaceRay.GetOrigin() - a;
				Vector2 w = b - a;

				float delta = Vector2::DotProduct(w, u);
				float result1 = Vector2::CrossProduct(w, v) / delta;
				float result2 = Vector2::DotProduct(v, u) / delta;

				if ((result2 >= 0.0f) && (result2 <= 1.0f) && (result1 > 0.0f) && (result1 < shortestDistance))
				{
					shortestDistance = result1;
					foundIndex = i;
				}
			}

			if (canContain)
			{
				result.SetContained(this);
				return true;
			}

			if (foundIndex >= 0 && shortestDistance <= bodySpaceRay.GetDistance())
			{
				result.Set(this, shortestDistance, pBodyAxes[foundIndex].GetNormal());
				return true;
			}

			return false;
		}

		bool PolygonCollider2D::CircleCastVertices(RayCast2D& bodySpaceRay, float radius, RayCastResult2D& result)
		{
			bool castHit = false;
			float sqrRadius = radius * radius;
			for (int i = 0; i < mBodyCount; i++)
			{
				castHit |= Collision2D::CircleRayCast(
					this,
					pBodyVertices[i],
					sqrRadius,
					bodySpaceRay,
					result);

				if (result.IsContained())
					return true;
			}

			return castHit;
		}

		void PolygonCollider2D::ConfigureColldier()
		{
			if (mBodyCount == 0)
			{
				SetWorldToBodyVertices(pBody, mWorldCount);
				mBodyCount = mWorldCount;
				CalculateAxes(pBodyVertices, pBodyAxes, mBodyCount);
				*pBodySpaceAABB = GetBoundsAABB(pBodyVertices, mBodyCount);
			}

			mInertia = CalculateInteria();
		}

		void PolygonCollider2D::ApplyBodyPosition()
		{
			for (int i = 0; i < mWorldCount; i++)
			{
				pWorldVertices[i] = pBody->BodyToWorldPoint(pBodyVertices[i]);
				pWorldAxes[i] = pBody->BodyToWorldAxis(pBodyAxes[i]);
			}

			*pWorldSpaceAABB = GetBoundsAABB(pWorldVertices, mWorldCount);
		}

		bool PolygonCollider2D::ColliderQueryPoint(Vector2 bodySpacePoint)
		{
			for (int i = 0; i < mBodyCount; i++)
			{
				if (Vector2::DotProduct(pBodyAxes[i].GetNormal(), bodySpacePoint) > pBodyAxes[i].GetWidth())
					return false;
			}

			return true;
		}

		bool PolygonCollider2D::ColliderQueryCircle(Vector2 bodySpaceOrigin, float radius)
		{
			int foundIndex = Collision2D::GetAxisPenetrationIndex(bodySpaceOrigin, radius, *this);
			if (foundIndex < 0)
				return false;

			Vector2 a = pBodyVertices[foundIndex];
			Vector2 b = pBodyVertices[(foundIndex + 1) % mBodyCount];
			Vector2 normal = pBodyAxes[foundIndex].GetNormal();
			float delta = Vector2::CrossProduct(normal, bodySpaceOrigin);

			if (delta > Vector2::CrossProduct(normal, a))
				return Collision2D::QueryPointInCircle(a, bodySpaceOrigin, radius);

			if (delta < Vector2::CrossProduct(normal, b))
				return Collision2D::QueryPointInCircle(b, bodySpaceOrigin, radius);

			return true;
		}

		bool PolygonCollider2D::ColliderRayCast(RayCast2D& bodySpaceRay, RayCastResult2D& result)
		{
			int foundIndex = NULL_PHASE;
			float maxDistance = 0.0f;
			float minDistance = FLT_MAX;
			bool canContain = true;

			for (int i = 0; i < mBodyCount; i++)
			{
				Vector2 normal = pBodyAxes[i].GetNormal();
				float projectedMagnitude = Vector2::DotProduct(normal, bodySpaceRay.GetOrigin()) - pBodyAxes[i].GetWidth();

				if (projectedMagnitude > 0.0f)
					canContain = false;

				normal.SetVector(-normal[0], -normal[1]); //flip the normal
				float slope = Vector2::DotProduct(normal, bodySpaceRay.GetDirection());
				if (slope == 0.0f)
					continue;

				float distance = projectedMagnitude / slope;
				if (slope > 0.0f)
				{
					if (distance > minDistance)
						return false;

					if (distance > maxDistance)
					{
						maxDistance = distance;
						foundIndex = i;
					}
				}
				else
				{
					if (distance < maxDistance)
						return false;

					if (distance < minDistance)
						minDistance = distance;
				}
			}

			if (canContain)
			{
				result.SetContained(this);
				return true;
			}

			if (foundIndex >= 0 && maxDistance <= bodySpaceRay.GetDistance())
			{
				result.Set(this, maxDistance, pBodyAxes[foundIndex].GetNormal());
				return true;
			}

			return false;
		}

		bool PolygonCollider2D::ColliderCircleCast(
			RayCast2D& bodySpaceRay,
			float radius,
			RayCastResult2D& result)
		{
			bool checkVertices = CircleCastVertices(bodySpaceRay, radius, result);
			bool checkEdges = CircleCastEdges(bodySpaceRay, radius, result);
			return checkVertices || checkEdges;

		}
		
		void PolygonCollider2D::Reset()
		{
			Collider2D::Reset();
			mBodyCount = 0;
			mWorldCount = 0;
		}

		//POOLING <--------------------------------------------

		PolygonCollider2D* POOL_PolygonCollider2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			PolygonCollider2D* obj = mPoolStack.top();
			obj->Reset();
			obj->SetPool(this);
			mPoolStack.pop();
			return obj;
		}

		void POOL_PolygonCollider2D::Deallocate(PolygonCollider2D* obj)
		{
			obj->Reset();
			obj->SetPool(nullptr);
			mPoolStack.push(obj);
		}
	}
}