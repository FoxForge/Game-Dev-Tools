#ifndef _SEQUENCEBTNODE_H_
#define _SEQUENCEBTNODE_H_

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
				class SequenceNode : public virtual ParentBehaviourNode
				{
				protected:
					std::string mName;
					std::vector<BehaviourNode*> mChildren;

				public:
					SequenceNode(std::string name);
					~SequenceNode();

					bool ContainsChild();
					void AddChild(BehaviourNode* child);
					virtual BEHAVIOUR_STATUS Tick(float deltaTime);
				};
			}
		}
	}
}

#endif // !_SEQUENCEBTNODE_H_

