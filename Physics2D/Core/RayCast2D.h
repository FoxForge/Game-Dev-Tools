#ifndef _RAYCAST2D_H_
#define _RAYCAST2D_H_

// we need to know the size of a vector2 to copy information, so sadly we cannot forward declare it here
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace Physics2D
	{
		using Maths::Vector2;

		class RayCast2D
		{
		private:
			Vector2 mOrigin;
			Vector2 mDireciton;
			Vector2 mInvDirection;
			float mDistance;
			bool mNegativeDirX;
			bool mNegativeDirY;

		public:
			RayCast2D(Vector2 origin, Vector2 end);
			RayCast2D(Vector2 origin, Vector2 direction, float distance);
			~RayCast2D() { ; }

			inline Vector2 GetOrigin() { return mOrigin; }
			inline Vector2 GetDirection() { return mDireciton; }
			inline Vector2 GetInvDirection() { return mInvDirection; }

			inline float GetDistance() { return mDistance; }
			inline bool IsNegativeDirectionX() { return mNegativeDirX; }
			inline bool IsNegativeDirectionY() { return mNegativeDirY; }
		};
	}
}


#endif // !_RAYCAST2D_H_

