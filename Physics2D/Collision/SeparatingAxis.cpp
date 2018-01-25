#include "SeparatingAxis.h"
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace Physics2D
	{
		SeparatingAxis::SeparatingAxis()
		{
			pNormal = new Vector2();
			mWidth = 0.0f;
		}

		SeparatingAxis::SeparatingAxis(Vector2 normal, float width)
		{
			pNormal = new Vector2(normal);
			mWidth = width;
		}

		SeparatingAxis::~SeparatingAxis()
		{
			delete pNormal;
		}

		Vector2& SeparatingAxis::GetNormal()
		{
			return *pNormal;
		}

		float SeparatingAxis::GetWidth() const
		{
			return mWidth;
		}
	}
}