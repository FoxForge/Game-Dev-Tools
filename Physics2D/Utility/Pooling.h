#ifndef _POOLING_H_
#define _POOLING_H_

#include <stack>

namespace Fox
{
	namespace Physics2D
	{		
		template<class T>
		class StackPool
		{
		protected:
			T* Create() { return new T(); }
			std::stack<T*> mPoolStack;

		public:
			// This deconstructor will be the same for all derived
			// Responsible for deleting all pointers
			~StackPool();
			virtual T* Allocate() = 0;
			virtual void Deallocate(T* obj) = 0;
		};

		// T must be of type PoolItem<T>
		template<class T>
		class StackPoolItem
		{
		private:
			StackPool<T>* pPool;

		public:
			StackPool<T>& GetPool() { return *pPool; }
			void SetPool(StackPool<T>* pool) { pPool = pool; }
			virtual void Reset() = 0;
		};

		class StackPoolUtility
		{
		public:
			template<class T>
			static void Free(StackPoolItem<T>* obj)
			{
				obj->GetPool().Deallocate(obj);
			}
		};
	}
}

#endif // !_POOLING_H_

