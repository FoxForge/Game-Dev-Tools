#ifndef _ACTIONBTNODE_H_
#define _ACTIONBTNODE_H_

#include "BehaviourNode.h"

#include <string>
#include <functional>

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				class ActionNode : public virtual BehaviourNode
				{
				private:
					std::string mName;
					std::function<BEHAVIOUR_STATUS(float)> mBehaviour;

				public:
					ActionNode(std::string name, std::function<BEHAVIOUR_STATUS(float)> behaviour);
					~ActionNode() { ; }

					BEHAVIOUR_STATUS Tick(float deltaTime);
				};

			}
		}
	}
}

#endif // !_ACTIONBTNODE_H_

