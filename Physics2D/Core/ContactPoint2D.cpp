#include "ContactPoint2D.h"

#include "..\..\Maths\Mathf.h"
#include "..\Utility\Physicsf.h"
#include "..\Collision\Manifold2D.h"

#include "Collider2D.h"
#include "Rigidbody2D.h"
#include <algorithm>

namespace Fox
{
	namespace Physics2D
	{
		ContactPoint2D::ContactPoint2D()
		{
			Reset();
		}

		ContactPoint2D& ContactPoint2D::Assign(Vector2 position, Vector2 normal, float penetration)
		{
			Reset();
			mPosition = position;
			mNormal = normal;
			mPenetration = penetration;
			return *this;
		}

		float ContactPoint2D::GetBiasDistance(float distance)
		{
			return RESOLVE_RATE * std::min(0.0f, distance + RESOLVE_COEFF);
		}

		float ContactPoint2D::ContactScalar(Rigidbody2D& a, Rigidbody2D& b, Vector2 normal)
		{
			// Switch division and multiplication with inv mass
			float massSum = a.GetInvMass() + b.GetInvMass();

			// Get the squared normals of contact
			float sqrdNormalA = Maths::Mathf::Square(Vector2::CrossProduct(mToA, normal));
			float sqrdNormalB = Maths::Mathf::Square(Vector2::CrossProduct(mToB, normal));

			// Calculate and return the scalar relative to the inertia
			return (massSum + (a.GetInvInertia() * sqrdNormalA) + (b.GetInvInertia() * sqrdNormalB));
		}

		Vector2 ContactPoint2D::GetRelativeVelocity(Rigidbody2D& a, Rigidbody2D& b)
		{
			return (mToALeft * a.GetAngularVelocity() + a.GetLinearVelocity()) - (mToBLeft * b.GetAngularVelocity() + b.GetLinearVelocity());
		}

		void ContactPoint2D::ApplyNormalBiasImpulse(Rigidbody2D& a, Rigidbody2D& b, float normalBiasImpulse)
		{
			Vector2 impulse = mNormal * normalBiasImpulse;

			// Add bias to B
			b.AddBias(impulse, mToA);

			// Flip bias to add opposite force to A
			impulse.SetVector(-impulse[0], -impulse[1]);
			a.AddBias(impulse, mToB);
		}

		void ContactPoint2D::ApplyContactImpulse(
			Rigidbody2D& a,
			Rigidbody2D& b,
			float normalImpulseMagnitude,
			float tangentImpulseMagnitude)
		{
			Vector2 impulseWorld = Vector2(normalImpulseMagnitude, tangentImpulseMagnitude);
			Vector2 impulseRelative = impulseWorld.Rotate(mNormal);

			// Add impulse to B
			b.AddImpulse(impulseRelative, mToB);

			// Flip impulse to add opposite force to A
			impulseRelative.SetVector(-impulseRelative[0], -impulseRelative[1]);
			a.AddImpulse(impulseRelative, mToA);
		}
		
		void ContactPoint2D::SetupCollision(Manifold2D& manifold)
		{
			Rigidbody2D& bodyA = manifold.GetColliderA().GetRigidBody();
			Rigidbody2D& bodyB = manifold.GetColliderB().GetRigidBody();

			// Setup direction vectors 
			mToA = mPosition - bodyA.GetPosition();
			mToB = mPosition - bodyB.GetPosition();

			// Setup rotated direction vectors 
			mToALeft = mToA.RotateLeft();
			mToBLeft = mToB.RotateLeft();

			// Setup scalars
			mNormalMassRatio = 1.0f / ContactScalar(bodyA, bodyB, mNormal);
			mTangentMassRatio = 1.0f / ContactScalar(bodyA, bodyB, mNormal.RotateLeft());

			// Setup bias
			mBias = GetBiasDistance(mPenetration);
			mCachedBiasImpulse = 0.0f;

			// Calculate restitution
			mRestitution = manifold.GetRestitution() * Vector2::DotProduct(mNormal, GetRelativeVelocity(bodyA, bodyB));
		}

		void ContactPoint2D::ApplyCachedSolution(Manifold2D& manifold)
		{
			// Apply the cached solution from previous solutions
			ApplyContactImpulse(
				manifold.GetColliderA().GetRigidBody(),
				manifold.GetColliderB().GetRigidBody(),
				mCachedNormalImpulse,
				mCachedTangentImpulse);
		}

		void ContactPoint2D::SolveCollision(Manifold2D& manifold, float elasticity)
		{
			// Get the bodies for the collision
			Rigidbody2D& bodyA = manifold.GetColliderA().GetRigidBody();
			Rigidbody2D& bodyB = manifold.GetColliderB().GetRigidBody();

			// Get the relative biased velocity - bias is used for having a minimum amount of force on collision
			Vector2 relativeBiasVelA = bodyA.GetBiasVelocity() + (mToALeft * bodyA.GetBiasRotation());
			Vector2 relativeBiasVelB = bodyB.GetBiasVelocity() + (mToBLeft * bodyB.GetBiasRotation());

			// Calculate the dot from the normal and relative velocities (bias magnitude)
			float velocityBiasM = Vector2::DotProduct((relativeBiasVelA - relativeBiasVelB), mNormal);

			// Calculate the bias impulse
			float biasImpulse = mNormalMassRatio * (velocityBiasM - mBias);

			// Clamp the bias impulse
			biasImpulse = std::max(-mCachedBiasImpulse, biasImpulse);

			// Cache the impulse bias (accumulate to solve small collisions smoothly)
			mCachedBiasImpulse += biasImpulse;

			// Apply the bias impulse
			ApplyNormalBiasImpulse(bodyA, bodyB, biasImpulse);

			// Calculate the normal impulse from relative velocity
			// Clamp the elasticity for elastic and non elastic collisions
			Vector2 relativeVel = GetRelativeVelocity(bodyA, bodyB);
			float impulseNormalM = mNormalMassRatio * (Vector2::DotProduct(relativeVel, mNormal) + (mRestitution * std::clamp(elasticity, 0.0f, 1.0f)));

			// Clamp the normal impulse with its cached value
			impulseNormalM = std::max(-mCachedNormalImpulse, impulseNormalM);

			// Cache the normal impulse (this should accumulate relative to elasticity)
			mCachedNormalImpulse += impulseNormalM;

			// Calculate the total tangent impulse with friction
			float impulseTangentLimit = manifold.GetFriction() * mCachedNormalImpulse;
			float impulseTangentM = Vector2::DotProduct(relativeVel, mNormal.RotateLeft()) * mTangentMassRatio;

			// Clamp the tangent impulse from this solution
			float tangentResult = std::clamp((mCachedTangentImpulse + impulseTangentM), -impulseTangentLimit, impulseTangentLimit);

			// Update the impulse to be the difference from the cache
			impulseTangentM = tangentResult - mCachedTangentImpulse;

			// Cache the tangent impulse (this should simply set the impulse as friction is not relative to elasticity)
			// Updating the cache with the most recent calculation
			mCachedTangentImpulse = tangentResult;

			// Apply the normal and tangent impulse
			ApplyContactImpulse(bodyA, bodyB, impulseNormalM, impulseTangentM);
		}

		void ContactPoint2D::Reset()
		{
			// Reset for pooling
			mToA.SetZero();
			mToB.SetZero();
			mNormal.SetZero();
			mToALeft.SetZero();
			mToBLeft.SetZero();
			mPosition.SetZero();

			mBias = 0.0f;
			mRestitution = 0.0f;
			mPenetration = 0.0f;
			mNormalMassRatio = 0.0f;
			mTangentMassRatio = 0.0f;
			mCachedBiasImpulse = 0.0f;
			mCachedNormalImpulse = 0.0f;
			mCachedTangentImpulse = 0.0f;
		}

		//POOLING <--------------------------------------------------

		ContactPoint2D* POOL_ContactPoint2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			ContactPoint2D* obj = mPoolStack.top();
			obj->Reset();
			obj->SetPool(this);
			mPoolStack.pop();
			return obj;
		}

		void POOL_ContactPoint2D::Deallocate(ContactPoint2D* obj)
		{
			obj->Reset();
			obj->SetPool(nullptr);
			mPoolStack.push(obj);
		}
	}
}
 