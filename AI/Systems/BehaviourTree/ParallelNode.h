#ifndef _PARALLELBTNODE_H_
#define _PARALLELBTNODE_H_

#include "ParentBehaviourNode.h"

#include <string>
#include <vector>

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				class ParallelNode : public virtual ParentBehaviourNode
				{
				private:
					size_t mNumberToFail;
					size_t mNumberToSucceed;
					std::string mName;
					std::vector<BehaviourNode*> mChildren;

				public:
					ParallelNode(
						std::string name,
						size_t numberToSucceed, 
						size_t numberToFail
					);

					~ParallelNode();

					bool ContainsChild();
					void AddChild(BehaviourNode* child);
					BEHAVIOUR_STATUS Tick(float deltaTime);
				};
			}
		}
	}
}

#endif // !_PARALLELBTNODE_H_

