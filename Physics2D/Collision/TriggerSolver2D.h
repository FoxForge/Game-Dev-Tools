#ifndef _TRGRSOLVER2D_H_
#define _TRGRSOLVER2D_H_

#include <set>

namespace Fox
{
	namespace Physics2D
	{
		class Manifold2D;
		class Collider2D;

		class TriggerSolver2D
		{
		private:
			std::set<Manifold2D*> mPreviousManifolds;
			static bool FindManifoldMatch(std::set<Manifold2D*>& set, Manifold2D* query);

		protected:
			std::set<Manifold2D*> mCurrentManifolds;
			std::set<Manifold2D*>::iterator mManifolderIter;

		public:
			TriggerSolver2D();
			~TriggerSolver2D(); // No need for virtual destructor here

			int GetCount() const;
			void FreeManifolds();
			void InvokeContacts();
			void Add(Manifold2D* manifold);
			void Clear();
		};
	}
}

#endif // !_TRGRSOLVER2D_H_

