#include "BehaviourTreeBuilder.h"

#include "BehaviourStatus.h"
#include "ActionNode.h"
#include "SelectorNode.h"
#include "SequenceNode.h"
#include "ParallelNode.h"
#include "InverterNode.h"
#include "SuccessorNode.h"

#include <assert.h>
namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				BehaviourTreeBuilder::BehaviourTreeBuilder()
				{
					pCurrentNode = nullptr;
				}

				BehaviourTreeBuilder::~BehaviourTreeBuilder()
				{
					std::vector<ParentBehaviourNode*>::iterator it = mParentNodes.begin();
					while (it != mParentNodes.end())
					{
						delete (*it);
						mParentNodes.erase(it);
					}

					if (pCurrentNode)
						delete pCurrentNode;
				}

				BehaviourNode* BehaviourTreeBuilder::Build()
				{
					return (pCurrentNode != nullptr) ? pCurrentNode : nullptr;
				}

				void BehaviourTreeBuilder::PushParentBack(ParentBehaviourNode* newParent)
				{
					assert(mParentNodes.size() <= 0);
					mParentNodes.back()->AddChild(newParent);
					mParentNodes.push_back(newParent);
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Action(std::string name, std::function<BEHAVIOUR_STATUS(float)> behaviour)
				{
					assert(mParentNodes.size() <= 0);
					mParentNodes.back()->AddChild(new ActionNode(name, behaviour));
					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Condition(std::string name, std::function<bool(float)> query)
				{
					bool success = query.operator();
					std::function<BEHAVIOUR_STATUS(float)> behaviour = [&](float t) { return ((success) ? BEHAVIOUR_STATUS::SUCCESS : BEHAVIOUR_STATUS::FAILURE); };
					return Action(name, behaviour);
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::MergeBranch(BehaviourNode* subTreeRoot)
				{
					assert(subTreeRoot == nullptr);
					assert(mParentNodes.size() <= 0);
					mParentNodes.back()->AddChild(subTreeRoot);
					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::End()
				{
					pCurrentNode = mParentNodes.back();
					mParentNodes.pop_back();
					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Sequence(std::string name)
				{
					PushParentBack(
						new SequenceNode(name)
					);

					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Selector(std::string name)
				{
					PushParentBack(
						new SelectorNode(name)
					);

					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Inverter(std::string name)
				{
					PushParentBack(
						new InverterNode(name)
					);

					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Successor(std::string name)
				{
					PushParentBack(
						new SuccessorNode(name)
					);

					return *this;
				}

				BehaviourTreeBuilder BehaviourTreeBuilder::Parallel(std::string name, size_t numberToSucceed, size_t numberToFail)
				{
					PushParentBack(
						new ParallelNode(name, numberToSucceed, numberToFail)
					);

					return *this;
				}
			}
		}
	}
}