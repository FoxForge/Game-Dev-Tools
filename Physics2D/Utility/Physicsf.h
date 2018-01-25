#ifndef _PHYSICSF_H_
#define _PHYSICSF_H_

#include <functional>

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		using Maths::Vector2;
		const float RESOLVE_COEFF = 0.1F;
		const float RESOLVE_RATE = 0.8F;
		const float DRAG_MOD = 0.01F;
		const float RESTITUTION = 0.5F;
		const float FRICTION = 0.8F;
		const float FIXED_DELTA_TIME = 0.02F;
		const float ELASTIC_MULT = 1.0F;
		const float NON_ELASTIC_MULT = 0.0F;
		const float DAMPING = 0.999F;
		const float MIN_DYNAMIC_MASS = 1.0F;
		const float MAX_MASS = 50000.0f;
		const float MASS_CONV_MULT = 0.01F;
		const float AABB_EXTENSION = 0.2F;

		const int ELASTIC_SOLVE_FTR = 1;
		const int NON_ELASTIC_SOLVE_FTR = 2;
		const int MAX_CONTACTS = 3;
		const int NULL_PHASE = -1;
		const int SOLVE_DELTA = 20;
		const int INIT_BUFFER_CAP = 256;


		//Forward declare for delegates
		class Rigidbody2D;
		typedef std::function<bool(Rigidbody2D)> RigidbodyFilter2D;
		typedef std::function<bool(Rigidbody2D, Rigidbody2D)> CollisionFilter2D;
		typedef std::function<void(Rigidbody2D&)> OnContact2D;

		class Physicsf
		{
		public:
			static Vector2 WorldToBodyPoint(Vector2 bodyPosition, Vector2 bodyFacing, Vector2 v);
			static Vector2 WorldToBodyDirection(Vector2 bodyFacing, Vector2 v);
			static Vector2 BodyToWorldPoint(Vector2 bodyPosition, Vector2 bodyFacing, Vector2 v);
			static Vector2 BodyToWorldDirection(Vector2 bodyFacing, Vector2 v);
		};
	}
}

#endif // !_PHYSICSF_H_

