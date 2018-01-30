#ifndef _SELECTBTNODE_H_
#define _SELECTBTNODE_H_

#include "ParentBehaviourNode.h"

#include <string>
#include <vector>

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				class SelectorNode : public virtual ParentBehaviourNode
				{
				protected:
					std::string mName;
					std::vector<BehaviourNode*> mChildren;

				public:
					SelectorNode(std::string name);
					~SelectorNode();

					bool ContainsChild();
					void AddChild(BehaviourNode* child);
					virtual BEHAVIOUR_STATUS Tick(float deltaTime);
				};
			}
		}
	}
}

#endif // !_SELECTBTNODE_H_

