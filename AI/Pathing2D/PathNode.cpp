#include "PathNode.h"

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			PathNode::PathNode()
			{
				// Set parent to null
				pParent = nullptr;

				// Create a full map of empty/null neighbours
				for (int i = 0; PathUtility::NeighbourIterFromInt(i, true); i++)
					mNeighbours.emplace(static_cast<NEIGHBOUR>(i), nullptr);
			}

			bool PathNode::GetNeighbour(NEIGHBOUR neighbour, PathNode* outNeighbour)
			{
				// Set pointer to null beforehand
				outNeighbour = nullptr;

				// Double check the map has been setup for the neighbour type, then get it's second value
				if (mNeighbours.count(neighbour))
					outNeighbour = mNeighbours[neighbour];
				
				// Only return true if a neighbour actually exists
				return outNeighbour != nullptr;
			}
		}
	}
}