#include "SelectorNode.h"
#include "BehaviourStatus.h"

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				SelectorNode::SelectorNode(std::string name)
				{
					mName = name;
				}

				SelectorNode::~SelectorNode()
				{
					std::vector<BehaviourNode*>::iterator it = mChildren.begin();
					while (it != mChildren.end())
					{
						delete (*it);
						mChildren.erase(it);
					}
				}

				bool SelectorNode::ContainsChild()
				{
					return mChildren.size() > 0;
				}

				void SelectorNode::AddChild(BehaviourNode* child)
				{
					mChildren.push_back(child);
				}

				BEHAVIOUR_STATUS SelectorNode::Tick(float deltaTime)
				{
					for (size_t i = 0; i < mChildren.size(); i++)
					{
						BEHAVIOUR_STATUS status = mChildren[i]->Tick(deltaTime);
						if (status != BEHAVIOUR_STATUS::FAILURE)
						{
							return status;
						}
					}

					return BEHAVIOUR_STATUS::FAILURE;
				}
			}
		}
	}
}