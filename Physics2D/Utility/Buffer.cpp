#include "Physicsf.h"
#include "Buffer.h"

namespace Fox
{
	namespace Physics2D
	{
		template<typename T>
		Buffer<T>::Buffer()
		{
			mCapacity = INIT_BUFFER_CAP;
			pElements = new T[INIT_BUFFER_CAP];
			mCount = 0;
		}

		template<typename T>
		Buffer<T>::~Buffer()
		{
			// Do not delete the pointers inside, that is not the buffer's job
			delete[] pElements;
		}

		template<typename T>
		T Buffer<T>::operator [] (const int i) const
		{
			return pElements[i];
		}

		template<typename T>
		T& Buffer<T>::operator [] (const int i)
		{
			return pElements[i];
		}

		template<typename T>
		void Buffer<T>::Add(T item)
		{
			if (mCount >= mCapacity)
			{
				// Double capacity - default behaviour
				T* newBuffer = new T[2 * mCapacity];

				// retain all items from old buffer
				for (i = 0; i < mCapacity; i++)
					newBuffer[i] = pElements[i];

				// Set doubled capacity - default behaviour
				mCapacity *= 2;

				// delete old buffer and set it to the new one
				// (we should not delete the pointers we want to transfer)
				delete[] pElements;
				pElements = newBuffer;
			}

			// Add item to the place desired in the buffer
			pElements[mCount++] = item;
		}

		template<typename T>
		void Buffer<T>::Add(T* items, int count)
		{
			// Can we NOT fit all the new items into the current buffer?
			if ((mCount + count) >= mCapacity)
			{
				// Keep doubling capacity until we can fit all items
				int newCapacity = mCapacity;
				while ((mCount + count) >= newCapacity)
					newCapacity *= 2;
				
				// retain all items from old buffer
				T* newBuffer = new T[newCapacity];
				for (i = 0; i < mCapacity; i++)
					newBuffer[i] = pElements[i];

				// add all new items to the buffer
				for (i = 0; i < count; i++)
					newBuffer[mCapacity + i] = items[i];

				// set new capacity
				mCapacity = newCapacity;

				// delete old buffer and set it to the new one 
				delete[] pElements;
				pElements = newBuffer;
			}
			else
			{
				// There is sufficient space in existing buffer
				// Simply add all new items to the buffer
				for (i = 0; i < count; i++)
					pElements[mCount + i] = items[i];
			}

			// Set the new count for either existing or resized buffer
			mCount += count;
		}
	}
}