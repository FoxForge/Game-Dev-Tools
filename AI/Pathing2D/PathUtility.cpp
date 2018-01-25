#include "PathUtility.h"
#include "PathNode.h"

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			bool PathUtility::NorthQuery(int& x, int& y, int i, int j)
			{
				return ((PositiveNorth()) ? (++y < j) : (--y >= 0));
			}

			bool PathUtility::EastQuery(int& x, int& y, int i, int j)
			{
				return ((PositiveEast()) ? (++x < i) : (--x >= 0));
			}

			bool PathUtility::SouthQuery(int& x, int& y, int i, int j)
			{
				return ((!PositiveNorth()) ? (++y < j) : (--y >= 0));
			}

			bool PathUtility::WestQuery(int& x, int& y, int i, int j)
			{
				return ((!PositiveEast()) ? (++x < i) : (--x >= 0));
			}

			bool PathUtility::NorthWestQuery(int& x, int& y, int i, int j)
			{
				return NorthQuery(x, y, i, j) && WestQuery(x, y, i, j);
			}

			bool PathUtility::NorthEastQuery(int& x, int& y, int i, int j)
			{
				return NorthQuery(x, y, i, j) && EastQuery(x, y, i, j);
			}

			bool PathUtility::SouthEastQuery(int& x, int& y, int i, int j)
			{
				return SouthQuery(x, y, i, j) && EastQuery(x, y, i, j);
			}

			bool PathUtility::SouthWestQuery(int& x, int& y, int i, int j)
			{
				return SouthQuery(x, y, i, j) && WestQuery(x, y, i, j);
			}

			NEIGHBOUR PathUtility::GetOppositeNeighbour(NEIGHBOUR neighbour)
			{
				// Opposite neighbour from a STRAIGHT neighbour type, return the cast
				if (!IsDiagonalNeighbour(neighbour))
					return static_cast<NEIGHBOUR>((int)(neighbour + 2) % (int)NEIGHBOUR::NORTH_WEST);

				// Opposite neighbour from a DIAGONAL neighbour type
				NEIGHBOUR opposite = static_cast<NEIGHBOUR>((int)(neighbour + 2) % ((int)NEIGHBOUR::SOUTH_WEST + 1));
				if (opposite < NEIGHBOUR::NORTH_WEST)
					opposite = static_cast<NEIGHBOUR>((int)opposite + 4);
				
				return opposite;
			}

			bool PathUtility::IsDiagonalNeighbourFromInt(int index)
			{
				// Diagonals are greater than or equal to 4 (NEIGHBOUR::NORTH_WEST)
				return index >= NEIGHBOUR::NORTH_WEST;
			}

			bool PathUtility::IsDiagonalNeighbour(NEIGHBOUR neighbour)
			{
				// Diagonals are greater than or equal to 4 (NEIGHBOUR::NORTH_WEST)
				return neighbour >= NEIGHBOUR::NORTH_WEST;
			}

			bool PathUtility::NeighbourIterFromInt(int index, bool diagonal)
			{
				// Used in FOR loops for simple iteration
				// Check an index against the clamp for desired neighbour type (straights or diagonals)
				return index < (diagonal ? NEIGHBOUR::SOUTH_WEST : NEIGHBOUR::WEST);
			}

			bool PathUtility::InvokeQuery(
				NEIGHBOUR neighbour,
				int& x,
				int& y,
				int i,
				int j)
			{
				// Find the correct query to match the neighbour type
				switch (neighbour)
				{
					// Straight Queries
					case Fox::AI::Pathing2D::NORTH: return NorthQuery(x, y, i, j);
					case Fox::AI::Pathing2D::EAST: return EastQuery(x, y, i, j);
					case Fox::AI::Pathing2D::SOUTH: return SouthQuery(x, y, i, j);
					case Fox::AI::Pathing2D::WEST: return WestQuery(x, y, i, j);

					// Diagonal Queries
					case Fox::AI::Pathing2D::NORTH_WEST: return NorthWestQuery(x, y, i, j);
					case Fox::AI::Pathing2D::NORTH_EAST: return NorthEastQuery(x, y, i, j);
					case Fox::AI::Pathing2D::SOUTH_EAST: return SouthEastQuery(x, y, i, j);
					case Fox::AI::Pathing2D::SOUTH_WEST: return SouthWestQuery(x, y, i, j);

					default: return false;
				}
			}

			void PathUtility::ConfigureNodes2D(PathNode** grid, unsigned int rows, unsigned int cols, bool configureDiagonals)
			{
				// Setup two integers for finding neighbour positions in the grid (use them as references)
				int x = 0;
				int y = 0;

				// Loop through rows
				for (unsigned int i = 0; i < rows; i++)
				{
					// Loop through columns
					for (unsigned int j = 0; j < cols; j++)
					{
						// Traverse each neighbour
						for (unsigned int k = 0; NeighbourIterFromInt(k, configureDiagonals); k++)
						{
							// Reset the grid references before applying changes to test for neighbours
							x = i;
							y = j;

							// Convert index to neighbour enum
							NEIGHBOUR neighbour = static_cast<NEIGHBOUR>(k);

							// Query a neighbour with the reference to it's position in the grid
							// If successful then a connection can be made to the neighbour
							if (InvokeQuery(neighbour, x, y, i, j))
								grid[i][j].SetNeighbour(neighbour, &grid[x][y]);

						}
					}
				}
			}
		}
	}
}