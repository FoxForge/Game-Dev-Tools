#include "SuccessorNode.h"
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
				SuccessorNode::SuccessorNode(std::string name)
				{
					mName = name;
				}

				SuccessorNode::~SuccessorNode()
				{
					if (pChild)
						delete pChild;
				}

				bool SuccessorNode::ContainsChild()
				{
					return pChild != nullptr;
				}

				void SuccessorNode::AddChild(BehaviourNode* child)
				{
					if (pChild)
						printf("Successor node has more than one child connected, it has been overidden. %s \n", mName);

					pChild = child;
				}

				BEHAVIOUR_STATUS SuccessorNode::Tick(float deltaTime)
				{
					assert(pChild == nullptr);
					BEHAVIOUR_STATUS status = pChild->Tick(deltaTime);

					// Always return success, irrelevant of the child tick behaviour
					return BEHAVIOUR_STATUS::SUCCESS;
				}
			}
		}
	}
}