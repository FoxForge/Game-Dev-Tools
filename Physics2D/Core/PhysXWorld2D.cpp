#include "PhysXWorld2D.h"

#include "..\Utility\Physicsf.h"
#include "..\Utility\Pooling.h"
#include "..\Utility\Buffer.h"

#include "..\Collision\Collision2D.h"
#include "..\Collision\TriggerSolver2D.h"
#include "..\Collision\CollisionSolver2D.h"
#include "..\Collision\Manifold2D.h"
#include "..\Collision\ContactType.h"

#include "..\Phases\BroadPhase2D.h"
#include "..\Phases\Dynamic2DTreeAABB.h"


#include "Collider2D.h"
#include "PhysXAABB.h"
#include "Rigidbody2D.h"
#include "PolygonCollider2D.h"
#include "ContactPoint2D.h"
#include "RayCast2D.h"
#include "RayCastResult2D.h"

namespace Fox
{
	namespace Physics2D
	{
		PhysXWorld2D::PhysXWorld2D(float worldDamping)
		{
			//core
			mDamping = worldDamping;
			mFixedDeltaTime = FIXED_DELTA_TIME;

			//solvers
			pPhysicalManifolds = new CollisionSolver2D(
				ELASTIC_SOLVE_FTR,
				NON_ELASTIC_SOLVE_FTR,
				ELASTIC_MULT,
				NON_ELASTIC_MULT,
				SOLVE_DELTA);

			pTriggerManifolds = new TriggerSolver2D();

			//phases
			pDynamicBroadPhase = new BroadPhase2D();
			pStaticBroadPhase = new Dynamic2DTreeAABB();
			
			//buffers
			pPhaseBuffer = new Buffer<Rigidbody2D*>();
			pQueryBuffer = new Buffer<Rigidbody2D*>();
			pQueryOutput = new Buffer<Rigidbody2D*>();

			//component pools
			pBodyPool = new POOL_Rigidbody2D();
			pManifoldPool = new POOL_Manifold2D();
			pContactPool = new POOL_ContactPoint2D();
			pPolygonPool = new POOL_PolygonCollider2D();
		}

		PhysXWorld2D::~PhysXWorld2D()
		{
			//solvers
			delete pPhysicalManifolds;
			delete pTriggerManifolds;

			//buffers
			delete pPhaseBuffer;
			delete pQueryBuffer;
			delete pQueryOutput;

			//component pools
			delete pBodyPool;
			delete pManifoldPool;
			delete pContactPool;
			delete pPolygonPool;
		}

		PolygonCollider2D& PhysXWorld2D::CreatePolygonCollider2DWorldSpace(
			Vector2 worldVertices[],
			const int length,
			bool isTrigger,
			float friction,
			float restitution,
			OnContact2D enter,
			OnContact2D stay,
			OnContact2D exit)
		{
			// Get a polygon collider instance from the pool
			PolygonCollider2D* polygon = pPolygonPool->Allocate();

			// Initialize the collider in world space
			polygon->InitFromWorldVertices(
				worldVertices,
				length,
				isTrigger,
				friction,
				restitution,
				enter,
				stay,
				exit);

			// Return polygon colldier value as reference
			return *polygon;
		}

		Rigidbody2D& PhysXWorld2D::CreateStaticBody(
			Vector2 position,
			float radians,
			Collider2D* collider)
		{
			// Get a body instance from the pool
			Rigidbody2D* body = pBodyPool->Allocate();

			// Initialize the body as static (no need for mass)
			body->InitStatic(position, radians, collider);

			// Add the body to the broadphase and the world
			AddBodyToWorld(*body);

			// Return body value as reference
			return *body;
		}

		Rigidbody2D& PhysXWorld2D::CreateDynamicBody(
			Vector2 position,
			float radians,
			float mass,
			Collider2D* collider,
			BODY_CONSTRAINT constraint)
		{
			// Get a body instance from the pool
			Rigidbody2D* body = pBodyPool->Allocate();

			// Initialize the body as dynamic
			body->InitDynamic(
				position,
				radians,
				mass,
				collider,
				constraint);

			// Add the body to the broadphase and the world
			AddBodyToWorld(*body);

			// Return body value as reference
			return *body;
		}

		Buffer<Rigidbody2D*> PhysXWorld2D::QueryPoint(Vector2 point, RigidbodyFilter2D filter)
		{
			// Clear the buffer ready to collect queries
			pQueryBuffer->Clear();

			// Query the phases which will add bodies to the buffer
			pStaticBroadPhase->QueryPoint(point, *pQueryBuffer);
			pDynamicBroadPhase->AddPhaseToBuffer(*pQueryBuffer);

			// Clear the output ready to transfer successful queries
			pQueryOutput->Clear();

			// Perform a "narrow phase like" query and add successful bodies to the output buffer
			for (int i = 0; i < pQueryBuffer->GetCount(); i++)
			{
				Rigidbody2D* body = (*pQueryBuffer)[i];
				if (Rigidbody2D::Filter(*body, filter) && body->QueryPoint(point))
				{
					pQueryOutput->Add(body);
				}
			}

			// Return the output buffer value
			return *pQueryOutput;
		}

		Buffer<Rigidbody2D*> PhysXWorld2D::QueryCircle(
			Vector2 origin,
			float radius,
			RigidbodyFilter2D filter)
		{
			// Clear the buffer ready to collect queries
			pQueryBuffer->Clear();

			// Query the phases which will add bodies to the buffer
			pStaticBroadPhase->QueryCircle(origin, radius, *pQueryBuffer);
			pDynamicBroadPhase->AddPhaseToBuffer(*pQueryBuffer);

			// Clear the output ready to transfer successful queries
			pQueryOutput->Clear();

			// Perform a "narrow phase like" query and add successful bodies to the output buffer
			for (int i = 0; i < pQueryBuffer->GetCount(); i++)
			{
				Rigidbody2D* body = (*pQueryBuffer)[i];
				if (Rigidbody2D::Filter(*body, filter) && body->QueryCircle(origin, radius))
				{
					pQueryOutput->Add(body);
				}
			}

			// Return the output buffer value
			return *pQueryOutput;
		}

		bool PhysXWorld2D::RayCast(
			RayCast2D& ray,
			RayCastResult2D& result,
			RigidbodyFilter2D filter)
		{
			// Clear the buffer ready to collect queries
			pQueryBuffer->Clear();

			// Query the phases which will add bodies to the buffer
			pStaticBroadPhase->QueryRayCast(ray, *pQueryBuffer);
			pDynamicBroadPhase->AddPhaseToBuffer(*pQueryBuffer);

			// Check buffer for a body which meets the ray cast hit requirement
			for (int i = 0; i < pQueryBuffer->GetCount(); i++)
			{
				Rigidbody2D* body = (*pQueryBuffer)[i];
				if (Rigidbody2D::Filter(*body, filter))
				{
					body->RayCast(ray, result);
					if (result.IsContained())
						return true;
				}
			}

			// Did the cast at least hit something?
			return result.IsValid();
		}

		bool PhysXWorld2D::CircleCast(
			RayCast2D& ray,
			float radius,
			RayCastResult2D& result,
			RigidbodyFilter2D filter)
		{
			// Clear the buffer ready to collect queries
			pQueryBuffer->Clear();

			// Query the phases which will add bodies to the buffer
			pStaticBroadPhase->QueryCircleCast(ray, radius, *pQueryBuffer);
			pDynamicBroadPhase->AddPhaseToBuffer(*pQueryBuffer);

			// Check buffer for a body which meets the ray cast hit requirement
			for (int i = 0; i < pQueryBuffer->GetCount(); i++)
			{
				Rigidbody2D* body = (*pQueryBuffer)[i];
				if (Rigidbody2D::Filter(*body, filter))
				{
					body->CircleCast(ray, radius, result);
					if (result.IsContained())
						return true;
				}
			}

			// Did the cast at least hit something?
			return result.IsValid();
		}

		void PhysXWorld2D::AddBodyToWorld(Rigidbody2D& body)
		{
			// Add to appropriate broadphase
			(body.IsStatic() ? pStaticBroadPhase : pDynamicBroadPhase)->AddBody(body);

			// Assign the body to this world and vice versa
			body.AssignWorld(this);
		}

		void PhysXWorld2D::RemoveBodyFromWorld(Rigidbody2D& body)
		{
			// Add to appropriate broadphase
			(body.IsStatic() ? pStaticBroadPhase : pDynamicBroadPhase)->RemoveBody(body);

			// Assign the body to this world and vice versa
			body.AssignWorld(nullptr);
		}

		void PhysXWorld2D::DestroyBody(Rigidbody2D& body)
		{
			// Make the body free its collider back to the world for pooling
			body.FreeCollider();

			// Remove the body from the broadphase and the world
			RemoveBodyFromWorld(body);

			// Deallocate the rigidbody back to the pool
			FreeBody(body);
		}

		// < MAIN WORLD UPDATE >
		void PhysXWorld2D::Update()
		{
			// Update all dynamic rigidbodies (use the broadphase to directly access them)
			for (int i = 0; i < pDynamicBroadPhase->GetCount(); i++)
				(*pDynamicBroadPhase)[i].Update();

			// Call the BroadPhase
			BroadPhase();

			// Solve all the collisions gathered from the manifolds created in the Narrow Phase
			UpdateCollisions();

			// Free all manifolds to be used again in future
			FreeManifolds();
		}

		void PhysXWorld2D::BroadPhase()
		{
			// Query every dynamic object
			for (int i = 0; i < pDynamicBroadPhase->GetCount(); i++)
			{
				// Create reference to dynamic body
				Rigidbody2D& body = (*pDynamicBroadPhase)[i];

				// Clear the buffer and start accumulating the successful queries against static objects
				pPhaseBuffer->Clear();

				// Populate the buffer from AABB queries with static objects
				pStaticBroadPhase->QueryOverlap(body.GetAABB(), *pPhaseBuffer);

				// Simply add all dynamic rigidbodies beyond this one to the buffer, to query them all overall (as all bodies will do this)
				for (int j = (i + 1); j < pDynamicBroadPhase->GetCount(); j++)
					pPhaseBuffer->Add(&(*pDynamicBroadPhase)[j]);

				// Check the buffer against the queried rigidbody
				QueryBufferNarrowPhase(body);
			}
		}

		void PhysXWorld2D::QueryBufferNarrowPhase(Rigidbody2D& query)
		{
			// Loop though the buffer
			for (int i = 0; i < pPhaseBuffer->GetCount(); i++)
			{
				// Are the queries allowed to collide? (Non-Self collision and at least one dynamic body)
				Rigidbody2D* test = (*pPhaseBuffer)[i];
				if (query.CanCollide(test) && test->CanCollide(&query))
					NarrowPhase(&query.GetCollider(), &test->GetCollider());
			}
		}

		void PhysXWorld2D::NarrowPhase(Collider2D* a, Collider2D* b)
		{
			// Are the two colliders AABB intersecting? No? return out immediately
			if (!a->GetWorldAABB().Intersect(b->GetWorldAABB()))
				return;

			// Attempt to create a manifold for the collision
			Manifold2D* manifold = Collision2D::Dispatch(*this, a, b);

			// If no succesful manifold was allocated, then we have a null pointer
			if (manifold == nullptr)
				return;

			// Is the manifold contact a trigger type? Add the manifold to the correct contact solver for the next update call
			(manifold->GetContactType() == CONTACT_TYPE::PHYSICAL ? pPhysicalManifolds : pTriggerManifolds)->Add(manifold);
		}

		void PhysXWorld2D::UpdateCollisions()
		{
			// Setup Collision
			pPhysicalManifolds->SetupCollisions();

			// Solve and apply ELASTIC collisions to the cache
			pPhysicalManifolds->SolveElasticCollisions();

			// Apply the cached solution of collision
			pPhysicalManifolds->ApplyCachedCollisionForces();

			// Solve and apply NON ELASTIC collisions to the cache
			pPhysicalManifolds->SolveNonElasticCollisions();

			// Invoke COLLISION Callbacks
			pPhysicalManifolds->InvokeContacts();

			// Invoke TRIGGER Callbacks
			pTriggerManifolds->InvokeContacts();
		}

		void PhysXWorld2D::FreeManifolds()
		{
			// Deallocate both physical and trigger manifolds (stack push instance)
			pPhysicalManifolds->FreeManifolds();
			pTriggerManifolds->FreeManifolds();

			// Clear physical and trigger manifold solvers to be populated next update
			pPhysicalManifolds->Clear();
			pTriggerManifolds->Clear();
		}

		void PhysXWorld2D::FreeBody(Rigidbody2D& body)
		{
			// Deallocate the body (stack push instance)
			pBodyPool->Deallocate(&body);
		}

		void PhysXWorld2D::FreeCollider(Collider2D* collider)
		{
			// Deallocate the collider (stack push instance)
			// Shape checking here / colldier type checking here
			pPolygonPool->Deallocate(reinterpret_cast<PolygonCollider2D*>(collider));
		}

		ContactPoint2D& PhysXWorld2D::AllocateContact()
		{
			// Allocate a contact (return a stack pop instance or create new)
			return *(pContactPool->Allocate());
		}

		Manifold2D& PhysXWorld2D::AllocateManifold()
		{
			// Allocate a manifold (return a stack pop instance or create new)
			return *(pManifoldPool->Allocate());
		}
	}
}