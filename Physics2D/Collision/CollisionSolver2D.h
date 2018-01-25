#ifndef _COLSOLVER2D_H_
#define _COLSOLVER2D_H_

#include "TriggerSolver2D.h"

namespace Fox
{
	namespace Physics2D
	{
		class CollisionSolver2D : public virtual TriggerSolver2D
		{
		private:
			int mSolveDelta;
			int mElasticFactor;
			int mNonElasticFactor;
			int mSolveElasticCount;
			int mSolveNonElasticCount;
			float mElasticMultiplier;
			float mNonElasticMultiplier;


		public:
			CollisionSolver2D(
				int elasticFactor,
				int nonElasticFactor,
				float elasticMultiplier,
				float nonElasticMultiplier,
				int solveDelta
			);

			~CollisionSolver2D();

			void SetupCollisions();
			void SolveElasticCollisions();
			void SolveNonElasticCollisions();
			void ApplyCachedCollisionForces();
		};
	}
}

#endif // !_COLSOLVER2D_H_

