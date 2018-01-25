#ifndef _BROADPHASE2D_H_
#define _BROADPHASE2D_H_

namespace Fox
{
	namespace Physics2D
	{
		// Forward declare Buffer
		template<class T> class Buffer;

		class Rigidbody2D;

		class BroadPhase2D
		{
		private:
			Rigidbody2D** pptrBodies;
			int mCapacity;
			int mCount;

		public:
			BroadPhase2D();

			virtual ~BroadPhase2D();

			virtual int GetCount();
			virtual Rigidbody2D& operator [] (const int i);

			void AddPhaseToBuffer(Buffer<Rigidbody2D*>& outBuffer);

			virtual void AddBody(Rigidbody2D& body);
			virtual void RemoveBody(Rigidbody2D& body);
		};

	}
}

#endif // !_BROADPHASE2D_H_

