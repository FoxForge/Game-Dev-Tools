#include <math.h>
#include "Vector4.h"
#include "Mathf.h"

namespace Fox
{
	namespace Maths
	{
		Vector4::Vector4()
		{
			mElement[0] = 0.0f;
			mElement[1] = 0.0f;
			mElement[2] = 0.0f;
			mElement[3] = 1.0f;
		}

		Vector4::Vector4(float x, float y, float z, float w)
		{
			mElement[0] = x;
			mElement[1] = y;
			mElement[2] = z;
			mElement[3] = w;
		}

		Vector4::Vector4(const Vector4& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
			mElement[2] = rhs[2];
			mElement[3] = rhs[3];
		}

		float Vector4::operator [] (const int i) const
		{
			return mElement[i];
		}

		float& Vector4::operator [] (const int i)
		{
			return mElement[i];
		}

		Vector4 Vector4::operator + (const Vector4& rhs) const
		{
			return Vector4(
				(*this)[0] + rhs[0],
				(*this)[1] + rhs[1],
				(*this)[2] + rhs[2],
				(*this)[3] + rhs[3]);
		}

		Vector4 Vector4::operator - (const Vector4& rhs) const
		{
			return Vector4(
				(*this)[0] - rhs[0],
				(*this)[1] - rhs[1],
				(*this)[2] - rhs[2],
				(*this)[3] - rhs[3]);
		}

		Vector4& Vector4::operator = (const Vector4& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
			mElement[2] = rhs[2];
			mElement[3] = rhs[3];
			return *this;
		}

		Vector4 Vector4::operator * (const Vector4& rhs) const
		{
			return Vector4(
				mElement[0] * rhs[0],
				mElement[1] * rhs[1],
				mElement[2] * rhs[2],
				mElement[3] * rhs[3]);
		}

		Vector4 Vector4::operator * (float scale) const
		{
			return Vector4(
				mElement[0] * scale,
				mElement[1] * scale,
				mElement[2] * scale,
				mElement[3] * scale
			);
		}

		float Vector4::Magnitude() const
		{
			return sqrtf(mElement[0] * mElement[0]
				+ mElement[1] * mElement[1]
				+ mElement[2] * mElement[2]
				+ mElement[3] * mElement[3]);
		}

		float Vector4::MagnitudeSqrd() const
		{
			return mElement[0] * mElement[0]
				+ mElement[1] * mElement[1]
				+ mElement[2] * mElement[2]
				+ mElement[3] * mElement[3];
		}

		float Vector4::DotProduct(const Vector4& rhs) const
		{
			return mElement[0] * rhs[0]
				+ mElement[1] * rhs[1]
				+ mElement[2] * rhs[2]
				+ mElement[3] * rhs[3];
		}

		void Vector4::Normalise()
		{
			float length = this->Magnitude();
			if (length > Mathf::VEC_EPSILON)
			{
				float invLen = 1.0f / length;
				mElement[0] *= invLen;
				mElement[1] *= invLen;
				mElement[2] *= invLen;
				mElement[3] *= invLen;
			}
		}

		Vector4 Vector4::CrossProduct(const Vector4& rhs) const
		{
			return Vector4(
				(mElement[1] * rhs[2] - mElement[2] * rhs[1]),
				(mElement[2] * rhs[0] - mElement[0] * rhs[2]),
				(mElement[0] * rhs[1] - mElement[1] * rhs[0]),
				1.0f // Set fourth dimension to one as default - cross product vector only 3rd and 7th dimension
			);
		}

		void Vector4::SetZero()
		{
			mElement[0] = mElement[1] = mElement[2] = mElement[3] = 0.0f;
		}
	}
}