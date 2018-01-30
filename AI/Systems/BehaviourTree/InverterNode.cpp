#include "InverterNode.h"
#include "BehaviourStatus.h"
#include <assert.h>

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				InverterNode::InverterNode(std::string name)
				{
					mName = name;
					pChild = nullptr;
				}

				InverterNode::~InverterNode()
				{
					if (pChild)
						delete pChild;
				}

				bool InverterNode::ContainsChild()
				{
					return pChild != nullptr;
				}

				void InverterNode::AddChild(BehaviourNode* child)
				{
					if (pChild)
						printf("Inverter node has more than one child connected, it has been overidden. %s \n", mName);

					pChild = child;
				}

				BEHAVIOUR_STATUS InverterNode::Tick(float deltaTime)
				{
					assert(pChild == nullptr);

					BEHAVIOUR_STATUS status = pChild->Tick(deltaTime);

					switch (status)
					{
					case BEHAVIOUR_STATUS::FAILURE:
						return BEHAVIOUR_STATUS::SUCCESS;

					case BEHAVIOUR_STATUS::SUCCESS:
						return BEHAVIOUR_STATUS::FAILURE;

					default:
						return BEHAVIOUR_STATUS::RUNNING;
					}
				}
			}
		}
	}
}