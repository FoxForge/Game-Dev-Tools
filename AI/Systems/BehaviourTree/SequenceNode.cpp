#include "SequenceNode.h"
#include "BehaviourStatus.h"

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				SequenceNode::SequenceNode(std::string name)
				{
					mName = name;
				}

				SequenceNode::~SequenceNode()
				{
					std::vector<BehaviourNode*>::iterator it = mChildren.begin();
					while (it != mChildren.end())
					{
						delete (*it);
						mChildren.erase(it);
					}
				}

				bool SequenceNode::ContainsChild()
				{
					return mChildren.size() > 0;
				}

				void SequenceNode::AddChild(BehaviourNode* child)
				{
					mChildren.push_back(child);
				}

				BEHAVIOUR_STATUS SequenceNode::Tick(float deltaTime)
				{
					for (size_t i = 0; i < mChildren.size(); i++)
					{
						BEHAVIOUR_STATUS status = mChildren[i]->Tick(deltaTime);
						if (status != BEHAVIOUR_STATUS::SUCCESS)
						{
							return status;
						}
					}

					return BEHAVIOUR_STATUS::SUCCESS;
				}
			}
		}
	}
}
