#include "Pooling.h"

namespace Fox
{
	namespace Physics2D
	{
		template<class T>
		StackPool<T>::~StackPool()
		{
			while (!mPoolStack.empty())
			{
				delete mPoolStack.top();
				mPoolStack.pop();
			}
		}

		/*Rigidbody2D* Pool_RigidBody2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			Rigidbody2D* obj = mPoolStack.top();
			(*obj).Reset();
			(*obj).SetPool(this);
			mPoolStack.pop();
			return obj;
		}

		Manifold2D* Pool_Manifold2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			Manifold2D* obj = mPoolStack.top();
			(*obj).Reset();
			(*obj).SetPool(this);
			mPoolStack.pop();
			return obj;
		}


		ContactPoint2D* Pool_ContactPoint2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			ContactPoint2D* obj = mPoolStack.top();
			(*obj).Reset();
			(*obj).SetPool(this);
			mPoolStack.pop();
			return obj;
		}


		PolygonCollider2D* Pool_PolygonCollider2D::Allocate()
		{
			if (mPoolStack.size() <= 0)
				return Create();

			PolygonCollider2D* obj = mPoolStack.top();
			(*obj).Reset();
			(*obj).SetPool(this);
			mPoolStack.pop();
			return obj;
		}


		void Pool_RigidBody2D::Deallocate(Rigidbody2D& obj)
		{
			obj.Reset();
			obj.SetPool(nullptr);
			mPoolStack.push(&obj);
		}

		void Pool_Manifold2D::Deallocate(Manifold2D& obj)
		{
			obj.Reset();
			obj.SetPool(nullptr);
			mPoolStack.push(&obj);
		}

		void Pool_ContactPoint2D::Deallocate(ContactPoint2D& obj)
		{
			obj.Reset();
			obj.SetPool(nullptr);
			mPoolStack.push(&obj);
		}

		void Pool_PolygonCollider2D::Deallocate(PolygonCollider2D& obj)
		{
			obj.Reset();
			obj.SetPool(nullptr);
			mPoolStack.push(&obj);
		}*/
	}
}