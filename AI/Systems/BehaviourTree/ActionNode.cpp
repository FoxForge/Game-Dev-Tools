#include "ActionNode.h"
#include "BehaviourStatus.h"

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				ActionNode::ActionNode(std::string name, std::function<BEHAVIOUR_STATUS(float)> behaviour)
				{
					mName = name;
					mBehaviour = behaviour;
				}

				BEHAVIOUR_STATUS ActionNode::Tick(float deltaTime)
				{
					return mBehaviour(deltaTime);
				}
			}
		}
	}
}