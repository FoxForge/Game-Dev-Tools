#include <math.h>
#include "Vector3.h"
#include "Mathf.h"

namespace Fox
{
	namespace Maths
	{
		Vector3::Vector3()
		{
			mElement[0] = 0.0f;
			mElement[1] = 0.0f;
			mElement[2] = 0.0f;
		}

		Vector3::Vector3(const Vector3& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
			mElement[2] = rhs[2];
		}

		Vector3::Vector3(float x, float y, float z)
		{
			mElement[0] = x; 
			mElement[1] = y; 
			mElement[2] = z;
		}

		float Vector3::operator [] (const int i) const
		{
			return mElement[i];
		}

		float& Vector3::operator [] (const int i)
		{
			return mElement[i];
		}

		Vector3 Vector3::operator + (const Vector3& rhs) const
		{
			return Vector3(
				(*this)[0] + rhs[0],
				(*this)[1] + rhs[1],
				(*this)[2] + rhs[2]);
		}

		Vector3 Vector3::operator - (const Vector3& rhs) const
		{
			return Vector3(
				(*this)[0] - rhs[0],
				(*this)[1] - rhs[1],
				(*this)[2] - rhs[2]);
		}

		Vector3& Vector3::operator = (const Vector3& rhs)
		{
			mElement[0] = rhs[0];
			mElement[1] = rhs[1];
			mElement[2] = rhs[2];
			return *this;
		}

		Vector3 Vector3::operator * (const Vector3& rhs) const
		{
			return Vector3(
				mElement[0] * rhs[0],
				mElement[1] * rhs[1],
				mElement[2] * rhs[2]);
		}

		Vector3 Vector3::operator * (float scale) const
		{
			return Vector3(
				mElement[0] * scale,
				mElement[1] * scale,
				mElement[2] * scale
			);
		}

		float Vector3::Magnitude() const
		{
			return sqrt(mElement[0] * mElement[0]
				+ mElement[1] * mElement[1] 
				+ mElement[2] * mElement[2]);
		}

		float Vector3::MagnitudeSqrd() const
		{
			return mElement[0] * mElement[0] 
				+ mElement[1] * mElement[1] 
				+ mElement[2] * mElement[2];
		}

		float Vector3::DotProduct(const Vector3& rhs) const
		{
			return mElement[0] * rhs[0] + 
				mElement[1] * rhs[1] + 
				mElement[2] * rhs[2];
		}

		Vector3 Vector3::Normalise()
		{
			float length = this->Magnitude();
			if (length > Mathf::VEC_EPSILON)
			{
				float invLen = 1.0f / length;
				mElement[0] *= invLen;
				mElement[1] *= invLen;
				mElement[2] *= invLen;
			}

			return *this;
		}

		Vector3 Vector3::CrossProduct(const Vector3& rhs) const
		{
			return Vector3(
				(mElement[1] * rhs[2] - mElement[2] * rhs[1]),
				(mElement[2] * rhs[0] - mElement[0] * rhs[2]),
				(mElement[0] * rhs[1] - mElement[1] * rhs[0])
			);
		}

		void Vector3::SetZero()
		{
			mElement[0] = mElement[1] = mElement[2] = 0.0f;
		}
	}
}