#include "CollisionSolver2D.h"
#include "Manifold2D.h"

namespace Fox
{
	namespace Physics2D
	{
		CollisionSolver2D::CollisionSolver2D(
			int elasticFactor,
			int nonElasticFactor,
			float elasticMultiplier,
			float nonElasticMultiplier,
			int solveDelta)
		{
			mSolveDelta = solveDelta;
			mElasticFactor = elasticFactor;
			mNonElasticFactor = nonElasticFactor;
			mElasticMultiplier = elasticMultiplier;
			mNonElasticMultiplier = nonElasticMultiplier;

			int solveDenominator = elasticFactor + nonElasticFactor;
			mSolveElasticCount = solveDelta * elasticFactor / solveDenominator;
			mSolveNonElasticCount = solveDelta * nonElasticFactor / solveDenominator;
		}

		CollisionSolver2D::~CollisionSolver2D() { ; }

		void CollisionSolver2D::SetupCollisions()
		{
			// Setup all collisions ready to be solved
			for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
			{
				(*mManifolderIter)->SetupCollision();
			}
		}

		void CollisionSolver2D::SolveElasticCollisions()
		{
			// Iterate over the collision steps
			for (int i = 0; i < mSolveElasticCount; i++)
			{
				for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
				{
					// Solve Collision with ELASTIC multiplier
					(*mManifolderIter)->SolveCollision(mElasticMultiplier);
				}
			}
		}

		void CollisionSolver2D::SolveNonElasticCollisions()
		{
			// Iterate over the collision steps
			for (int i = 0; i < mSolveNonElasticCount; i++)
			{
				for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
				{
					// Solve Collision with NON-ELASTIC multiplier
					(*mManifolderIter)->SolveCollision(mNonElasticMultiplier);
				}
			}
		}

		void CollisionSolver2D::ApplyCachedCollisionForces()
		{
			// Apply the cache from previous solutions on each collision
			for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
			{
				(*mManifolderIter)->ApplyCachedSolution();
			}
		}
	}
}