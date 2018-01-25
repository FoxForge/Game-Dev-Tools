#ifndef _VECTOR3_H_
#define _VECTOR3_H_

namespace Fox
{
	namespace Maths
	{
		class Vector3
		{
		private:
			float mElement[3];

		public:
			Vector3();
			Vector3(const Vector3& rhs);
			Vector3(float x, float y, float z);

			~Vector3() { ; }

			void SetZero();

			float Magnitude() const;
			float MagnitudeSqrd() const;
			float DotProduct(const Vector3& rhs) const;
			float operator [] (const int i) const;
			float& operator [] (const int i);

			Vector3 Normalise();
			Vector3 operator * (float scale) const;
			Vector3 operator + (const Vector3& rhs) const;
			Vector3 operator - (const Vector3& rhs) const;
			Vector3 operator * (const Vector3& rhs) const;
			Vector3 CrossProduct(const Vector3& rhs) const;
			Vector3& operator = (const Vector3& rhs);

			inline float GetX() { return mElement[0]; }
			inline float GetY() { return mElement[1]; }
			inline float GetZ() { return mElement[2]; }

			inline void SetVector(float x, float y, float z)
			{
				mElement[0] = x;
				mElement[1] = y;
				mElement[2] = z;
			}
		};
	}
}

#endif // !_VECTOR3_H_


