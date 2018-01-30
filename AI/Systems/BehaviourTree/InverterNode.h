#ifndef _INVERTERBTNODE_H_
#define _INVERTERBTNODE_H_

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
				class InverterNode : public virtual ParentBehaviourNode
				{
				private:
					std::string mName;
					BehaviourNode* pChild;

				public:
					InverterNode(std::string name);
					~InverterNode();

					bool ContainsChild();
					void AddChild(BehaviourNode* child);
					BEHAVIOUR_STATUS Tick(float deltaTime);
				};
			}
		}
	}
}

#endif // !_INVERTERBTNODE_H_

