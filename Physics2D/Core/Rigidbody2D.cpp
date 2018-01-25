#include "Rigidbody2D.h"

#include "..\..\Maths\Vector2.h"
#include "..\Utility\Physicsf.h"
#include "..\Collision\SeparatingAxis.h"
#include "BodyType.h"
#include "PhysXAABB.h"
#include "PhysXWorld2D.h"
#include "Collider2D.h"
#include "BodyConstraint.h"
#include "RayCast2D.h"
#include "RayCastResult2D.h"
#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		Rigidbody2D::Rigidbody2D()
		{
			Reset();
			mPhaseID = NULL_PHASE;
			pAABB = new PhysXAABB(0.0f, 0.0f, 0.0f, 0.0f);
		}

		Rigidbody2D::~Rigidbody2D()
		{
			delete pAABB;
		}

		bool Rigidbody2D::Filter(Rigidbody2D body, RigidbodyFilter2D filter)
		{
			if (filter == nullptr)
				return true;

			return filter.operator()(body);
		}

		bool Rigidbody2D::IsStatic() const
		{
			return mBodyType == BODY_TYPE::STATIC;
		}

		int Rigidbody2D::GetPhaseID() const
		{
			return mPhaseID;
		}

		float Rigidbody2D::GetAngle() const
		{
			return mAngle;
		}

		float Rigidbody2D::GetMass() const
		{
			return mMass;
		}

		float Rigidbody2D::GetInvMass() const
		{
			return mInvMass;
		}

		float Rigidbody2D::GetInertia() const
		{
			return mInertia;
		}

		float Rigidbody2D::GetInvInertia() const
		{
			return mInvInertia;
		}

		float Rigidbody2D::GetBiasRotation() const
		{
			return mBiasRotation;
		}

		float Rigidbody2D::GetAngularVelocity() const
		{
			return mAngularVelocity;
		}

		void Rigidbody2D::SetPhaseID(int id)
		{
			mPhaseID = id;
		}

		void Rigidbody2D::SetLinearVelocity(Vector2 velocity)
		{
			mLinearVelocity = velocity;
		}

		void Rigidbody2D::SetAngularVelocity(float torque)
		{
			mAngularVelocity = torque;
		}

		BODY_TYPE Rigidbody2D::GetBodyType() const
		{
			return mBodyType;
		}

		Vector2 Rigidbody2D::GetLinearVelocity() const
		{
			return mLinearVelocity;
		}

		Vector2 Rigidbody2D::GetPosition() const
		{
			return mPosition;
		}

		Vector2 Rigidbody2D::GetFacing() const
		{
			return mFacing;
		}
		
		Vector2 Rigidbody2D::GetBiasVelocity() const
		{
			return mBiasVelocity;
		}

		PhysXAABB& Rigidbody2D::GetAABB() const
		{
			return *pAABB;
		}

		PhysXWorld2D& Rigidbody2D::GetWorld() const
		{
			return *pWorld;
		}

		Collider2D& Rigidbody2D::GetCollider() const
		{
			return *pCollider;
		}

		void Rigidbody2D::InitDynamic(
			Vector2 position,
			float radians,
			float mass,
			Collider2D* collider,
			BODY_CONSTRAINT constraint)
		{
			// Generic Initialize for rigidbody
			Init(position, radians, collider);

			// Calculates the body's dynamics
			SetDynamic(mass, constraint);
		}

		void Rigidbody2D::InitStatic(
			Vector2 position,
			float radians,
			Collider2D* colldier)
		{
			// Generic Initialize for rigidbody
			Init(position, radians, colldier);

			// Calcualtes the body as static
			SetStatic();
		}

		void Rigidbody2D::Init(
			Vector2 position,
			float radians,
			Collider2D* collider)
		{
			//directly set position, angle and calculate facing vector
			mPosition = position;
			mAngle = radians;
			mFacing = Vector2::EulerToVector2(radians);

			// Assign the collider to this rigidbody
			pCollider = collider;
			pCollider->AssignBody(this);

			// Tell the collider to update its points after body set
			pCollider->UpdatePosition();

			// Update the AABB now the collider has updated
			UpdateAABB();
		}
		
		void Rigidbody2D::Set(Vector2 position, float radians)
		{
			//directly set position, angle and calculate facing vector
			mPosition = position;
			mAngle = radians;
			mFacing = Vector2::EulerToVector2(radians);

			// Immediately clear forces and force the position to update
			ClearForces();

			// Tell the collider to update its points after body translation
			pCollider->UpdatePosition();

			// Update the AABB now the collider has updated
			UpdateAABB();
		}

		void Rigidbody2D::SetDynamic(float mass, BODY_CONSTRAINT constraint)
		{
			// Setup the dynamic properties of the rigidbody
			// Clamp the mass with a scaling (scaling multiplier makes the mass values friendly for use - not tiny values)
			mMass = std::clamp(mMass * MASS_CONV_MULT, MASS_CONV_MULT, MAX_MASS);

			// Store the contraint
			mBodyConstraint = constraint;

			// Calculate inertia (for rotations)
			mInertia = pCollider->GetInertia() * mMass;

			// Setup Inverse properties to prevent reccurant calculation
			mInvMass = 1.0f / mMass;
			mInvInertia = 1.0f / mInertia;

			// Finally set body type
			mBodyType = BODY_TYPE::DYNAMIC;
		}

		void Rigidbody2D::SetStatic()
		{
			// Set static properties of rigidbody, everything can be zero
			mMass = 0.0f;
			mInertia = 0.0f;
			mInvMass = 0.0f;
			mInvInertia = 0.0f;

			// Finally set body type
			mBodyType = BODY_TYPE::STATIC;
		}

		void Rigidbody2D::Update()
		{
			// Integrate Physics
			Integrate();

			// Tell the collider to update its points after body translation
			pCollider->UpdatePosition();

			// Update the AABB now the collider has updated
			UpdateAABB();
		}

		void Rigidbody2D::AssignWorld(PhysXWorld2D* world)
		{
			pWorld = world;
		}

		void Rigidbody2D::SetCollisionFilter(CollisionFilter2D filter)
		{
			mCollisionFilter = filter;
		}

		void Rigidbody2D::AddTorque(float torque)
		{
			mTorqueCache += torque;
		}

		void Rigidbody2D::AddForce(Vector2 force)
		{
			mForceCache += force;
		}

		void Rigidbody2D::AddForceRelativeTo(Vector2 force, Vector2 point)
		{
			
			mForceCache += force;

			// Update angular velocity after calculating rotation
			mTorqueCache += Vector2::CrossProduct(mPosition - point, force);
		}

		void Rigidbody2D::AddImpulse(Vector2 force, Vector2 direction)
		{
			// Update linear velocity from collision
			mLinearVelocity += force * mInvMass;

			// Update angular velocity after calculating rotation
			mAngularVelocity -= Vector2::CrossProduct(force, direction) * mInvInertia;
		}

		void Rigidbody2D::AddBias(Vector2 force, Vector2 direction)
		{
			// Update bias velocity from collision
			mBiasVelocity += force * mInvMass;

			// Update bias angular velocity after calculating rotation
			mBiasRotation -= Vector2::CrossProduct(force, direction) * mInvInertia;
		}

		void Rigidbody2D::UpdateAABB()
		{
			// The collider has been updated before and now we have new AABB bounds
			// Dont need to delete and change the pointer, just the value at the pointer location
			// Handle the AABB allocation through constructor and deconstructor
			*pAABB = PhysXAABB(
				pCollider->GetWorldAABB().GetTop(),
				pCollider->GetWorldAABB().GetBottom(),
				pCollider->GetWorldAABB().GetLeft(),
				pCollider->GetWorldAABB().GetRight()
			);
		}

		void Rigidbody2D::ClearForces()
		{
			mTorqueCache = 0.0f;
			mBiasRotation = 0.0f;
			mForceCache.SetZero();
			mBiasVelocity.SetZero();
		}

		void Rigidbody2D::Integrate()
		{
			// Apply resistances and world damping to the velocites
			mLinearVelocity *= pWorld->GetWorldDamping();
			mAngularVelocity *= pWorld->GetWorldDamping();

			// Calculate the acceleration
			Vector2 linearAcceleration = mForceCache * mInvMass;
			float angularAcceleration = mTorqueCache * mInvInertia;

			// Integrate the half the force as part of the mid-point algorithm (acceleration -> velocity)
			IntegrateMidPointAcceleration(linearAcceleration, angularAcceleration);

			// Integrate the velocity (velocity -> position)
			IntegrateVelocity();

			// Integrate forces again, so the next update will integrate smoothly (acceleration -> velocity)
			IntegrateMidPointAcceleration(linearAcceleration, angularAcceleration);

			// Clear the forces for acceleration
			ClearForces();
		}

		void Rigidbody2D::IntegrateMidPointAcceleration(Vector2 force, float torque)
		{
			// This can be done irrespective of body constraints
			// Integrate half of the acceleration force and torque to velocity
			mLinearVelocity += force * pWorld->GetFixedDeltaTime() * ACC_MOD;
			mAngularVelocity -= torque * pWorld->GetFixedDeltaTime() * ACC_MOD;
		}

		void Rigidbody2D::IntegrateVelocity()
		{
			// Integrate velocity based on contraints:
			// Integrate the velocity to position and rotaiton (angle)
			// Update the facing direciton based on new rotation (angle)
			switch (mBodyConstraint)
			{
			case BODY_CONSTRAINT::NONE:
				mPosition += mLinearVelocity * pWorld->GetFixedDeltaTime() + mBiasVelocity;
				mAngle += mAngularVelocity * pWorld->GetFixedDeltaTime() + mBiasRotation;
				mFacing = Vector2::EulerToVector2(mAngle);
				break;

			case BODY_CONSTRAINT::POSITION:
				mAngle += mAngularVelocity * pWorld->GetFixedDeltaTime() + mBiasRotation;
				mFacing = Vector2::EulerToVector2(mAngle);
				break;

			case BODY_CONSTRAINT::ROTATION:
				mPosition += mLinearVelocity * pWorld->GetFixedDeltaTime() + mBiasVelocity;
				break;

			default: break;
			}
		}

		void Rigidbody2D::FreeCollider()
		{
			if (pWorld == nullptr || pCollider == nullptr)
				return;

			// Call the world to free our collider and put it back in the pooling system
			pWorld->FreeCollider(pCollider);

			// Null our collider - body is not responsible for allocation or deallocation
			pCollider = nullptr;
		}

		bool Rigidbody2D::CanCollide(Rigidbody2D* other) const
		{
			if ((this == other) || (IsStatic() && other->IsStatic()))
				return false;

			if (mCollisionFilter != nullptr)
				return mCollisionFilter.operator()((*this), *other);

			return true;
		}

		bool Rigidbody2D::QueryPoint(Vector2 point)
		{
			if (!pAABB->QueryPoint(point))
				return false;

			Vector2 bodyPoint = BodyToWorldPoint(point);
			if (pCollider->QueryPoint(bodyPoint))
				return true;

			return false;
		}

		bool Rigidbody2D::QueryCircle(Vector2 origin, float radius)
		{
			if (!pAABB->QueryCircleApprox(origin, radius))
				return false;

			Vector2 bodySpaceOrigin = WorldToBodyPoint(origin);
			if (pCollider->QueryCircle(bodySpaceOrigin, radius))
				return true;

			return false;
		}

		bool Rigidbody2D::RayCast(RayCast2D& ray, RayCastResult2D& result)
		{
			if (!pAABB->RayCast(ray))
				return false;

			RayCast2D bodySpaceRay = WorldToBodyRay(ray);
			if (pCollider->RayCast(ray, result))
				if (result.IsContained())
					return true;

			if (&result.GetBody() == this)
				result.SetNormal(BodyToWorldDirection(result.GetNormal()));

			return result.IsValid();
		}

		bool Rigidbody2D::CircleCast(
			RayCast2D& ray,
			float radius,
			RayCastResult2D& result)
		{
			if (pAABB->CircleCastApprox(ray, radius))
				return false;

			RayCast2D bodySpaceRay = WorldToBodyRay(ray);
			if (pCollider->CircleCast(ray, radius, result))
				if (result.IsContained())
					return true;

			if (&result.GetBody() == this)
				result.SetNormal(BodyToWorldDirection(result.GetNormal()));

			return result.IsValid();
		}

		RayCast2D Rigidbody2D::WorldToBodyRay(RayCast2D& ray)
		{
			return RayCast2D(
				WorldToBodyPoint(ray.GetOrigin()),
				WorldToBodyDirection(ray.GetDirection()),
				ray.GetDistance()
			);
		}

		Vector2 Rigidbody2D::WorldToBodyDirection(Vector2 direction)
		{
			return Physicsf::WorldToBodyDirection(mFacing, direction);
		}

		Vector2 Rigidbody2D::BodyToWorldDirection(Vector2 direction)
		{
			return Physicsf::BodyToWorldDirection(mFacing, direction);
		}

		Vector2 Rigidbody2D::WorldToBodyPoint(Vector2 point)
		{
			return Physicsf::WorldToBodyPoint(mPosition, mFacing, point);
		}

		Vector2 Rigidbody2D::BodyToWorldPoint(Vector2 point)
		{
			return Physicsf::BodyToWorldPoint(mPosition, mFacing, point);
		}

		SeparatingAxis Rigidbody2D::BodyToWorldAxis(SeparatingAxis axis)
		{
			Vector2 normal = axis.GetNormal().Rotate(mFacing);
			float width = Vector2::DotProduct(normal, mPosition) + axis.GetWidth();
			return SeparatingAxis(normal, width);
		}
		
		void Rigidbody2D::Reset()
		{
			// Reset for pooling
			pWorld = nullptr;
			mCollisionFilter = nullptr;

			mMass = 0.0f;
			mAngle = 0.0f;
			mInvMass = 0.0f;
			mInertia = 0.0f;
			mTorqueCache = 0.0f;
			mInvInertia = 0.0f;
			mBiasRotation = 0.0f;

			mForceCache.SetZero();
			mBiasVelocity.SetZero();
			mLinearVelocity.SetZero();
		}

		//POOLING ------------------------------------------------------------------

		Rigidbody2D* POOL_Rigidbody2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			Rigidbody2D* obj = mPoolStack.top();
			obj->Reset();
			obj->SetPool(this);
			mPoolStack.pop();
			return obj;
		}

		void POOL_Rigidbody2D::Deallocate(Rigidbody2D* obj)
		{
			obj->Reset();
			obj->SetPool(nullptr);
			mPoolStack.push(obj);
		}
	}
}