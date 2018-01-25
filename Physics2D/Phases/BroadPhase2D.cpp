#include "BroadPhase2D.h"
#include "..\Core\Rigidbody2D.h"
#include "..\Utility\Buffer.h"
#include "..\Utility\Physicsf.h"

namespace Fox
{
	namespace Physics2D
	{
		BroadPhase2D::BroadPhase2D()
		{
			mCapacity = INIT_BUFFER_CAP;
			pptrBodies = new Rigidbody2D*[INIT_BUFFER_CAP];
			mCount = 0;
		}

		BroadPhase2D::~BroadPhase2D()
		{
			delete[] pptrBodies;
		}

		int BroadPhase2D::GetCount()
		{
			return mCount;
		}

		Rigidbody2D& BroadPhase2D::operator[] (const int i)
		{
			return *pptrBodies[i];
		}

		void BroadPhase2D::AddPhaseToBuffer(Buffer<Rigidbody2D*>& outBuffer)
		{
			outBuffer.Add(pptrBodies, mCount);
		}

		void BroadPhase2D::AddBody(Rigidbody2D& body)
		{
			// Same behaviour as Buffer<T>
			if (mCount >= mCapacity)
			{
				Rigidbody2D** newBuffer = new Rigidbody2D*[2 * mCapacity];
				for (int i = 0; i < mCapacity; i++)
					newBuffer[i] = pptrBodies[i];

				mCapacity *= 2;
				delete[] pptrBodies;
				pptrBodies = newBuffer;
			}

			pptrBodies[mCount] = &body;
			body.SetPhaseID(mCount++);
		}

		void BroadPhase2D::RemoveBody(Rigidbody2D& body)
		{
			int index = body.GetPhaseID();
			int lastIndex = mCount - 1;
			if (index < lastIndex)
			{
				Rigidbody2D* lastBody = pptrBodies[lastIndex];
				lastBody->SetPhaseID(NULL_PHASE);
				pptrBodies[lastIndex] = nullptr;
				pptrBodies[index] = lastBody;
				lastBody->SetPhaseID(index);
			}

			mCount--;
		}
	}

}