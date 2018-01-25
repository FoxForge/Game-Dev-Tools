#include "RayCast2D.h"

namespace Fox
{
	namespace Physics2D
	{
		RayCast2D::RayCast2D(Vector2 origin, Vector2 end)
		{
			Vector2 delta = origin - end;
			mOrigin = origin;
			mDireciton = delta.GetNormalized();
			mDistance = delta.Magnitude();
			mNegativeDirX = mDireciton.GetX() < 0.0f;
			mNegativeDirY = mDireciton.GetY() < 0.0f;
			mInvDirection = Vector2(
				(1.0f / mDireciton.GetX()),
				(1.0f / mDireciton.GetY()));
		}

		RayCast2D::RayCast2D(Vector2 origin, Vector2 direction, float distance)
		{
			mOrigin = origin;
			mDireciton = direction;
			mDistance = distance;
			mNegativeDirX = mDireciton.GetX() < 0.0f;
			mNegativeDirY = mDireciton.GetY() < 0.0f;
			mInvDirection = Vector2(
				(1.0f / mDireciton.GetX()),
				(1.0f / mDireciton.GetY()));
		}
	}
}