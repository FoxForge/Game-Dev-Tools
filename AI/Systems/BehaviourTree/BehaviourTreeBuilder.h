#ifndef _BEHAVTREEBUILDER_H_
#define _BEHAVTREEBUILDER_H_

#include <vector>
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
				// Forward declare node types
				class BehaviourNode;
				class ParentBehaviourNode;

				enum BEHAVIOUR_STATUS : unsigned short;

				class BehaviourTreeBuilder
				{
				private:
					BehaviourNode* pCurrentNode;
					std::vector<ParentBehaviourNode*> mParentNodes;
					void PushParentBack(ParentBehaviourNode* newParent);

				public:
					BehaviourTreeBuilder();
					~BehaviourTreeBuilder();

					BehaviourNode* Build();

					BehaviourTreeBuilder Action(std::string name, std::function<BEHAVIOUR_STATUS(float)> behaviour);
					BehaviourTreeBuilder Condition(std::string name, std::function<bool(float)> query);
					BehaviourTreeBuilder MergeBranch(BehaviourNode* subTreeRoot);
					BehaviourTreeBuilder End();

					BehaviourTreeBuilder Sequence(std::string name);
					BehaviourTreeBuilder Selector(std::string name);
					BehaviourTreeBuilder Inverter(std::string name);
					BehaviourTreeBuilder Successor(std::string name);
					BehaviourTreeBuilder Parallel(
						std::string name,
						size_t numberToSucceed,
						size_t numberToFail
					);
				};

			}
		}
	}
}

#endif // !_BEHAVTREEBUILDER_H_

