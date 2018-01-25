#ifndef _BUFFER_H_
#define _BUFFER_H_

namespace Fox
{
	namespace Physics2D
	{
		template<typename T>
		class Buffer
		{
		private:
			T* pElements;
			int mCount;
			int mCapacity;
			int i;

		public:
			Buffer();
			~Buffer();

			T operator [] (const int i) const;
			T& operator [] (const int i);
			void Add(T item);
			void Add(T* items, int count);
		
			inline int GetCount() { return mCount; }
			inline void Clear() { mCount = 0; }
		};
	}
}

#endif // !_BUFFER_H_

