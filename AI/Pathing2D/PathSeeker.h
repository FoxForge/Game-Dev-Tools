#ifndef _PATH_SEEKER_H_
#define _PATH_SEEKER_H_

#include "PathNode.h"
#include <vector>
#include <algorithm>
#include <functional>

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			class PathSeeker
			{
			public:
				PathSeeker(bool includeDiagonal, float straightCost = 1.0f, float diagCost = 1.41f);

				~PathSeeker() { ; }

				inline void SetOnNode(PathNode* node) { pOnNode = node; }

				void ResetContainers();

				bool GetPath(
					PathNode* goal,
					std::vector<PathNode*>& pathBuffer,
					std::function<bool(PathNode*)> excludeSearch = nullptr,
					std::map<std::function<bool(PathNode*)>, float> costSearch = std::map<std::function<bool(PathNode*)>, float>(),
					bool accumulateCostSearch = false);

			private:
				const unsigned int CYCLE_SAFE = 900;

				bool mIncludeDiagonal;
				float mDiagCost;
				float mDeltaCost;
				float mStraightCost;
				PathNode* pOnNode;

				std::vector<PathNode*> mOpenedNodes;
				std::vector<PathNode*> mClosedNodes;
				std::vector<PathNode*> mPathNodes;
				std::vector<PathNode*>::iterator mIter;
				std::map<std::function<bool(PathNode*)>, float>::iterator costIter;

				bool GetBestNode(PathNode* outNode);
				void FillPathVector(PathNode* goal);
				void AddPathToBuffer(std::vector<PathNode*>& pathBuffer);
				void ApplyScoring(PathNode* current, PathNode* neighbour, PathNode* goal, float cost);
			};
		}
	}
}


#endif // !_PATH_SEEKER_H_


