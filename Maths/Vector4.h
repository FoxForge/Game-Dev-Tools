#ifndef _VECTOR4_H_
#define _VECTOR4_H_

namespace Fox
{
	namespace Maths
	{
		class Vector4
		{
		private:
			float mElement[4];

		public:
			Vector4();
			Vector4(const Vector4& rhs);
			Vector4(float x, float y, float z, float w = 1.0f);

			~Vector4() { ; }

			void SetZero();
			void Normalise();

			float Magnitude() const;
			float MagnitudeSqrd() const;
			float DotProduct(const Vector4& rhs) const;
			float operator [] (const int i) const;
			float& operator [] (const int i);

			Vector4 operator * (float scale) const;
			Vector4 operator + (const Vector4& rhs) const;
			Vector4 operator - (const Vector4& rhs) const;
			Vector4 operator * (const Vector4& rhs) const;
			Vector4 CrossProduct(const Vector4& rhs) const;
			Vector4& operator = (const Vector4& rhs);

			inline float GetX() { return mElement[0]; }
			inline float GetY() { return mElement[1]; }
			inline float GetZ() { return mElement[2]; }
			inline float GetW() { return mElement[3]; }

			inline void SetVector(float x, float y, float z, float w)
			{
				mElement[0] = x;
				mElement[1] = y;
				mElement[2] = z;
				mElement[3] = w;
			}
		};
	}
}

#endif // !_VECTOR4_H_


