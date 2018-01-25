#ifndef _SEPAXIS_H_
#define _SEPAXIS_H_

namespace Fox
{
	namespace Maths
	{
		class Vector2;
	}

	namespace Physics2D
	{
		using Maths::Vector2;

		class SeparatingAxis
		{
		private:
			Vector2* pNormal;
			float mWidth;

		public:
			SeparatingAxis();
			SeparatingAxis(Vector2 normal, float width);
			~SeparatingAxis();
			
			float GetWidth() const;
			Vector2& GetNormal();
		};
	}
}

#endif // !_SEPAXIS_H_

