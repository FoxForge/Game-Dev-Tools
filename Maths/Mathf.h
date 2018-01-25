#ifndef _MATHF_H_
#define _MATHF_H_

namespace Fox
{
	namespace Maths
	{
		class Mathf
		{
		public:
			static const float VEC_EPSILON;
			inline static float Square(float n) { return n*n; }
		};
	}
}


#endif // !_MATHF_H_

