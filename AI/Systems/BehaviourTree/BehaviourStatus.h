#ifndef _BEHAVSTATUS_H_
#define _BEHAVSTATUS_H_

namespace Fox
{
	namespace AI
	{
		namespace Systems
		{
			namespace BehaviourTree
			{
				enum BEHAVIOUR_STATUS : unsigned short
				{
					FAILURE = 0,
					SUCCESS,
					RUNNING
				};
			}
		}
	}
}

#endif // !_BEHAVSTATUS_H_

