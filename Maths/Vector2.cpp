#include <math.h>
#include "Vector2.h"
#include "Mathf.h"

namespace Fox 
{
	namespace Maths
	{
		Vector2::Vector2()
		{
			mElement[0] = 0.0f;
			mElement[1] = 0.0f;
		}

		Vector2::Vector2(const Vector2& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
		}

		Vector2::Vector2(float x, float y)
		{
			mElement[0] = x; 
			mElement[1] = y;
		}

		float Vector2::operator [] (const int i) const
		{
			return mElement[i];
		}

		float& Vector2::operator [] (const int i)
		{
			return mElement[i];
		}

		Vector2 Vector2::operator + (const Vector2& rhs) const
		{
			return Vector2(
				(*this)[0] + rhs[0],
				(*this)[1] + rhs[1]
			);
		}

		Vector2 Vector2::operator - (const Vector2& rhs) const
		{
			return Vector2(
				(*this)[0] - rhs[0],
				(*this)[1] - rhs[1]
			);
		}

		Vector2& Vector2::operator = (const Vector2& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
			return *this;
		}

		Vector2& Vector2::operator += (const Vector2& rhs)
		{
			mElement[0] += rhs[0];
			mElement[1] += rhs[1];
			return *this;
		}

		Vector2& Vector2::operator -= (const Vector2& rhs)
		{
			mElement[0] -= rhs[0];
			mElement[1] -= rhs[1];
			return *this;
		}

		Vector2 Vector2::operator * (const Vector2& rhs) const
		{
			return Vector2(
				mElement[0] * rhs[0],
				mElement[1] * rhs[1]
			);
		}

		Vector2& Vector2::operator *= (float scale)
		{
			mElement[0] *= scale;
			mElement[1] *= scale;
			return *this;
		}

		Vector2 Vector2::operator * (float scale) const
		{
			return Vector2(
				mElement[0] * scale,
				mElement[1] * scale
			);
		}

		float Vector2::Magnitude() const
		{
			return sqrt(mElement[0]
				* mElement[0] 
				+ mElement[1] 
				* mElement[1]);
		}

		float Vector2::MagnitudeSqrd() const
		{
			return mElement[0]
				* mElement[0]
				+ mElement[1]
				* mElement[1];
		}

		float Vector2::DotProduct(const Vector2& rhs) const
		{
			return mElement[0] * rhs[0]
				+ mElement[1] * rhs[1];
		}

		Vector2 Vector2::Normalise()
		{
			float length = this->Magnitude();

			if (length > Mathf::VEC_EPSILON)
			{
				float invLen = 1.0f / length;
				mElement[0] *= invLen;
				mElement[1] *= invLen;
			}

			return *this;
		}

		Vector2 Vector2::GetNormalized()
		{
			Vector2 v(*this);
			v.Normalise();
			return v;
		}

		float Vector2::CrossProduct(const Vector2& rhs) const
		{
			// Cross product "scalar"
			return mElement[0] * rhs[1]
				- mElement[1] * rhs[0];
		}

		void Vector2::SetZero()
		{
			mElement[0] = 0.0f;
			mElement[1] = 0.0f;
		}

		void Vector2::FlipX()
		{
			mElement[1] = -mElement[1];
		}

		void Vector2::FlipY()
		{
			mElement[0] = -mElement[0];
		}

		void Vector2::SwapXY()
		{
			float tempX = mElement[0];
			mElement[0] = mElement[1];
			mElement[1] = tempX;
		}

		// Static functions
		Vector2 Vector2::EulerToVector2(float radians)
		{
			return Vector2(cos(radians), sin(radians));
		}

		Vector2 Vector2::RotateLeft()
		{
			Vector2 v(*this);
			float x = v[0];
			v[0] = -v[1];
			v[1] = x;
			return v;
		}

		Vector2 Vector2::Rotate(Vector2 b)
		{
			Vector2 v(*this);
			float x = v[0];
			float y = v[1];
			v[0] = x * b[0] - y * b[1];
			v[1] = y * b[0] + x * b[1];
			return v;
		}

		Vector2 Vector2::InvRotate(Vector2 b)
		{
			Vector2 v(*this);
			float x = v[0];
			float y = v[1];
			v[0] = x * b[0] + y * b[1];
			v[1] = y * b[0] - x * b[1];
			return v;
		}

		float Vector2::CrossProduct(Vector2& a, Vector2& b)
		{
			return a.CrossProduct(b);
		}
		
		float Vector2::DotProduct(Vector2& a, Vector2& b)
		{
			return a.DotProduct(b);
		}
	}
}