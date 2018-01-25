#include "TriggerSolver2D.h"
#include "Manifold2D.h"
#include "..\Core\Collider2D.h"

namespace Fox
{
	namespace Physics2D
	{
		TriggerSolver2D::TriggerSolver2D() { ; }

		TriggerSolver2D::~TriggerSolver2D() { ; }

		int TriggerSolver2D::GetCount() const
		{
			return mCurrentManifolds.size();
		}

		void TriggerSolver2D::InvokeContacts()
		{
			bool currentCacheExists = mCurrentManifolds.size() > 0;
			bool previousCacheExists = mPreviousManifolds.size() > 0;

			// Check PREVIOUS Cache for current matches
			for (mManifolderIter = mPreviousManifolds.begin(); mManifolderIter != mPreviousManifolds.end(); mManifolderIter++)
			{
				Manifold2D* manifold = (*mManifolderIter);
				if (!currentCacheExists || !FindManifoldMatch(mCurrentManifolds, manifold))
				{
					// Invoke Exit
					manifold->InvokeContactExit2D();
				}
				else
				{
					// Invoke stay
					manifold->InvokeContactStay2D();
				}
			}

			// Check CURRENT cache for previous matches
			for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
			{
				Manifold2D* manifold = (*mManifolderIter);
				if (!previousCacheExists || !FindManifoldMatch(mPreviousManifolds, manifold))
				{
					// Invoke Enter
					manifold->InvokeContactEnter2D();
				}
			}
			
			// TRANSFER manifold cache after clearing
			mPreviousManifolds.clear();
			for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
			{
				Manifold2D* manifold = (*mManifolderIter);
				mPreviousManifolds.emplace(manifold);
			}
		}

		void TriggerSolver2D::Add(Manifold2D* manifold)
		{
			// Expose emplacing
			mCurrentManifolds.emplace(manifold);
		}

		void TriggerSolver2D::Clear()
		{
			// Expose clearing
			mCurrentManifolds.clear();
		}

		void TriggerSolver2D::FreeManifolds()
		{
			for (mManifolderIter = mCurrentManifolds.begin(); mManifolderIter != mCurrentManifolds.end(); mManifolderIter++)
			{
				Manifold2D* manifold = (*mManifolderIter);
				manifold->GetPool().Deallocate(manifold);
			}
		}

		bool TriggerSolver2D::FindManifoldMatch(std::set<Manifold2D*>& set, Manifold2D* query)
		{
			std::set<Manifold2D*>::iterator it = set.begin();
			Collider2D* qColA = &query->GetColliderA();
			Collider2D* qColB = &query->GetColliderB();

			while (it != set.end())
			{
				Manifold2D* current = (*it);
				Collider2D* colA = &current->GetColliderA();
				Collider2D* colB = &current->GetColliderB();

				// If either of the addresses match for opposite collders then the manifolds match
				if (((colA == qColA) && (colB == qColB)) || ((colA == qColB) && (colB == qColA)))
					return true;
				
				it++;
			}

			return false;
		}
	}
}