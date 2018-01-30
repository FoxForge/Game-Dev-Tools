#include "ParallelNode.h"
#include "BehaviourStatus.h"

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				ParallelNode::ParallelNode(
					std::string name,
					size_t numberToSucceed,
					size_t numberToFail)
				{
					mName = name;
					mNumberToSucceed = numberToSucceed;
					mNumberToFail = numberToFail;
				}

				ParallelNode::~ParallelNode()
				{
					std::vector<BehaviourNode*>::iterator it = mChildren.begin();
					while (it != mChildren.end())
					{
						delete (*it);
						mChildren.erase(it);
					}
				}

				bool ParallelNode::ContainsChild()
				{
					return mChildren.size() > 0;
				}

				void ParallelNode::AddChild(BehaviourNode* child)
				{
					mChildren.push_back(child);
				}

				BEHAVIOUR_STATUS ParallelNode::Tick(float deltaTime)
				{
					size_t successCount = 0;
					size_t failCount = 0;

					for (size_t i = 0; i < mChildren.size(); i++)
					{
						BEHAVIOUR_STATUS status = mChildren[i]->Tick(deltaTime);
						switch (status)
						{
						case BEHAVIOUR_STATUS::FAILURE:
							++failCount; break;

						case BEHAVIOUR_STATUS::SUCCESS:
							++successCount; break;

						default: break;
						}
					}

					if (mNumberToSucceed > 0 && successCount >= mNumberToSucceed)
						return BEHAVIOUR_STATUS::SUCCESS;

					if (mNumberToFail > 0 && failCount >= mNumberToFail)
						return BEHAVIOUR_STATUS::FAILURE;

					return BEHAVIOUR_STATUS::RUNNING;
				}
			}
		}
	}
}