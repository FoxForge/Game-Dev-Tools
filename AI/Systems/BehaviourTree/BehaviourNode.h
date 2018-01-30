#ifndef _BEHAVNODE_H_
#define _BEHAVNODE_H_

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				enum BEHAVIOUR_STATUS : unsigned short;

				class BehaviourNode
				{
				public:
					virtual ~BehaviourNode() { ; }
					virtual BEHAVIOUR_STATUS Tick(float deltaTime) = 0;
				};
			}
		}
	}
}


#endif // !_BEHAVNODE_H_

