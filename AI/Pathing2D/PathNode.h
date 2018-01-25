#ifndef _PATH_NODE_H_
#define _PATH_NODE_H_

#include <map>
#include "PathUtility.h"
#include "..\..\Maths\Vector2.h"

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			struct NodeData
			{
				bool isRouted;
				bool isClosed;
				bool isOpened;
				bool isPath;
				float heuristic;
				float distance;
				float cost;

				// Default construct for default init of data
				NodeData() :
					isRouted(),
					isClosed(),
					isOpened(),
					isPath(),
					heuristic(),
					distance(),
					cost()
				{}
			};

			class PathNode
			{
			public:
				PathNode();

				~PathNode() { ; }

				bool GetNeighbour(NEIGHBOUR neighbour, PathNode* outNeighbour);

				inline PathNode* GetParent() { return pParent; }
				inline NodeData& GetNodeData() { return mNodeData; }
				inline Fox::Maths::Vector2 GetPosition() { return mPosition; }
				inline void SetParent(PathNode* parent) { pParent = parent; }
				inline void SetNeighbour(NEIGHBOUR neighbour, PathNode* node) { mNeighbours[neighbour] = node; }

				

			private:
				PathNode* pParent;
				NodeData mNodeData;
				Fox::Maths::Vector2 mPosition;
				std::map<NEIGHBOUR, PathNode*> mNeighbours;
			};
		}
	}
}



#endif // !_PATH_NODE_H_


