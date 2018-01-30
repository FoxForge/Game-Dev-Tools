#ifndef _SUCCESSORBTNODE_H_
#define _SUCCESSORBTNODE_H_

#include "ParentBehaviourNode.h"

#include <string>

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				class SuccessorNode : public virtual ParentBehaviourNode
				{
				private:
					std::string mName;
					BehaviourNode* pChild;

				public:
					SuccessorNode(std::string name);
					~SuccessorNode();

					bool ContainsChild();
					void AddChild(BehaviourNode* child);
					BEHAVIOUR_STATUS Tick(float deltaTime);
				};
			}
		}
	}
}

#endif // !_SUCCESSORBTNODE_H_


