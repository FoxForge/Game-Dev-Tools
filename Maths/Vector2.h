#ifndef _VECTOR2_H_
#define _VECTOR2_H_

namespace Fox
{
	namespace Maths
	{
		class Vector2
		{

		private:
			float mElement[2];

		public:
			Vector2();
			Vector2(float x, float y);
			Vector2(const Vector2& rhs);

			~Vector2() { ; }

			void SetZero();

			void FlipY();
			void FlipX();
			void SwapXY();

			float Magnitude() const;
			float MagnitudeSqrd() const;
			float DotProduct(const Vector2& rhs) const;
			float CrossProduct(const Vector2& rhs) const;
			float operator [] (const int i) const;
			float& operator [] (const int i);

			Vector2 Normalise();
			Vector2 GetNormalized();

			Vector2 operator * (float scale) const;
			Vector2 operator + (const Vector2& rhs) const;
			Vector2 operator - (const Vector2& rhs) const;
			Vector2 operator * (const Vector2& rhs) const;
			
			Vector2& operator *= (float scale);
			Vector2& operator = (const Vector2& rhs);
			Vector2& operator += (const Vector2& rhs);
			Vector2& operator -= (const Vector2& rhs);

			inline float GetX() { return mElement[0]; }
			inline float GetY() { return mElement[1]; }

			inline void SetVector(float x, float y)
			{
				mElement[0] = x;
				mElement[1] = y;
			}

			Vector2 RotateLeft();
			Vector2 Rotate(Vector2 b);
			Vector2 InvRotate(Vector2 b);

			static Vector2 EulerToVector2(float radians);
			static float CrossProduct(Vector2& a, Vector2& b);
			static float DotProduct(Vector2& a, Vector2& b);
		};
	}
}

#endif // !_VECTOR2_H_

