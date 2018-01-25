#ifndef _PATH_UTILITY_H_
#define _PATH_UTILITY_H_

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			// Forward declare PathNode
			class PathNode;

			enum NEIGHBOUR
			{
				NORTH = 0,
				EAST,
				SOUTH,
				WEST,
				NORTH_WEST,
				NORTH_EAST,
				SOUTH_EAST,
				SOUTH_WEST
			};

			enum PathOrientation
			{
				POSITIVE_N_E = 0,
				POSITIVE_S_E,
				POSITIVE_N_W,
				POSITIVE_S_W,
			};

			class PathUtility
			{
			public:

				static NEIGHBOUR GetOppositeNeighbour(NEIGHBOUR neigbhour);
				static bool IsDiagonalNeighbourFromInt(int index);
				static bool IsDiagonalNeighbour(NEIGHBOUR neighbour);
				static bool NeighbourIterFromInt(int index, bool diagonal);

				static bool InvokeQuery(
					NEIGHBOUR neighbour,
					int& x,
					int& y,
					int i,
					int j);

				static void ConfigureNodes2D(PathNode** grid, unsigned int rows, unsigned int cols, bool configureDiagonals);

				inline static void SetOrientation(PathOrientation or) { orientation = or ; }
				inline static PathOrientation GetOrientation() { return orientation; }

			private:
				static PathOrientation orientation;

				static bool NorthQuery(int& x, int& y, int i, int j);
				static bool EastQuery(int& x, int& y, int i, int j);
				static bool SouthQuery(int& x, int& y, int i, int j);
				static bool WestQuery(int& x, int& y, int i, int j);

				static bool NorthWestQuery(int& x, int& y, int i, int j);
				static bool NorthEastQuery(int& x, int& y, int i, int j);
				static bool SouthEastQuery(int& x, int& y, int i, int j);
				static bool SouthWestQuery(int& x, int& y, int i, int j);

				inline static bool PositiveNorth()
				{
					return (orientation == PathOrientation::POSITIVE_N_E
						|| orientation == PathOrientation::POSITIVE_N_W);
				}

				inline static bool PositiveEast()
				{
					return (orientation == PathOrientation::POSITIVE_N_E
						|| orientation == PathOrientation::POSITIVE_S_E);
				}
			};
		}
	}
}


#endif // !_PATH_UTILITY_H_

