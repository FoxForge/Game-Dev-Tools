#ifndef _PARENTBEHAVNODE_H_
#define _PARENTBEHAVNODE_H_

#include "BehaviourNode.h"

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				class ParentBehaviourNode : public virtual BehaviourNode
				{
				public:
					virtual ~ParentBehaviourNode() { ; }
					virtual void AddChild(BehaviourNode* child) = 0;
					virtual bool ContainsChild() = 0;
				};
			}
		}
	}
}

#endif // !_PARENTBEHAVNODE_H_

