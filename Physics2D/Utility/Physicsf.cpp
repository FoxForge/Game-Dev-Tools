#include "Physicsf.h"
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace Physics2D
	{
		Vector2 Physicsf::WorldToBodyPoint(Vector2 bodyPosition, Vector2 bodyFacing, Vector2 v)
		{
			return (v - bodyPosition).InvRotate(bodyFacing);
		}

		Vector2 Physicsf::WorldToBodyDirection(Vector2 bodyFacing, Vector2 v)
		{
			return v.InvRotate(bodyFacing);
		}

		Vector2 Physicsf::BodyToWorldPoint(Vector2 bodyPosition, Vector2 bodyFacing, Vector2 v)
		{
			return v.Rotate(bodyFacing) + bodyPosition;
		}

		Vector2 Physicsf::BodyToWorldDirection(Vector2 bodyFacing, Vector2 v)
		{
			return v.Rotate(bodyFacing);
		}
	}
}