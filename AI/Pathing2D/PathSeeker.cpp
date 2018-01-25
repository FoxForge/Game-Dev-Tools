#include "PathSeeker.h"
#include "PathUtility.h"

namespace Fox
{
	namespace AI
	{
		namespace Pathing2D
		{
			PathSeeker::PathSeeker(
				bool includeDiagonal,
				float straightCost,
				float diagCost)
			{
				// Give the seeker values for search properties
				mDiagCost = diagCost;
				mStraightCost = straightCost;
				mIncludeDiagonal = includeDiagonal;
				mDeltaCost = abs(straightCost - diagCost);
			}

			void PathSeeker::ResetContainers()
			{
				// For every node still stored by pointer, reset the respective bools
				for (auto& node : mOpenedNodes)
					node->GetNodeData().isOpened = false;
				
				for (auto& node : mClosedNodes)
					node->GetNodeData().isClosed = false;

				for (auto& node : mPathNodes)
					node->GetNodeData().isPath = false;

				// Clear the vectors of all pointers
				mOpenedNodes.clear();
				mClosedNodes.clear();
				mPathNodes.clear();
			}

			bool PathSeeker::GetBestNode(PathNode* outNode)
			{
				// If there are no opened nodes then we cannot select, return false
				if (mOpenedNodes.size() < 1)
					return false;

				// Find the best heuristic value from all the opened nodes
				mIter = mOpenedNodes.begin();
				outNode = (*mIter);
				mIter++;

				while (mIter != mOpenedNodes.end())
				{
					// Pass back out the best node found with the lowest heuristic
					if ((*mIter)->GetNodeData().heuristic < outNode->GetNodeData().heuristic)
						outNode = (*mIter);

					mIter++;
				}

				// Return true, we found a node
				return true;
			}

			void PathSeeker::FillPathVector(PathNode* goal)
			{
				PathNode* node = goal;
				node->GetNodeData().isPath = true;
				mPathNodes.push_back(node);

				// Starting at to goal, follow path backwards until we find the start node
				while (node != pOnNode)
				{
					if (node->GetParent() == nullptr)
						break;

					// Follow parent path
					node = node->GetParent();

					// Add to path vector
					node->GetNodeData().isPath = true;
					mPathNodes.push_back(node);

					// Remove from closed vector
					node->GetNodeData().isClosed = false;
					std::remove(mClosedNodes.begin(), mClosedNodes.end(), node);
				}

				// Reverse path vector to begin at start (onNode)
				std::reverse(mPathNodes.begin(), mPathNodes.end());
			}

			void PathSeeker::AddPathToBuffer(std::vector<PathNode*>& pathBuffer)
			{
				// Clear the buffer given and pass it all nodes in the path vector
				pathBuffer.clear();
				for (auto& node : mPathNodes)
					pathBuffer.push_back(node);
			}

			void PathSeeker::ApplyScoring(
				PathNode* current,
				PathNode* neighbour,
				PathNode* goal,
				float cost)
			{
				// Find the size of deltas and calculate heuristic of neighbour based on distance
				neighbour->GetNodeData().cost = current->GetNodeData().cost + cost;
				float dx = abs((*goal).GetPosition().GetX() - (*neighbour).GetPosition().GetX());
				float dy = abs((*goal).GetPosition().GetY() - (*neighbour).GetPosition().GetY());
				neighbour->GetNodeData().distance = std::max(dx, dy) + mDeltaCost * std::min(dx, dy);

				// Heuristic = cost + distance
				neighbour->GetNodeData().heuristic = neighbour->GetNodeData().cost + neighbour->GetNodeData().distance;
			}


			bool PathSeeker::GetPath(
				PathNode* goal,
				std::vector<PathNode*>& pathBuffer,
				std::function<bool(PathNode*)> excludeSearch,
				std::map<std::function<bool(PathNode*)>, float> costSearch,
				bool accumulateCostSearch)
			{
				// Return false if search is not setup initially
				if (goal == nullptr || pOnNode == nullptr)
					return false;

				// If we are already on the goal then quick return out and clear the buffer
				// We will only clear the buffer on a successful return, otherwise keep what's in the buffer passed to us
				if (pOnNode == goal)
				{
					pathBuffer.clear();
					return true;
				}

				// Setup search bools to avoid recalculation
				bool hasExclude = excludeSearch != nullptr;
				bool hasCostMap = costSearch.size() > 0;

				// Set cycles and reset containers from any previous searches
				unsigned int cycles = 0;
				ResetContainers();

				// Begin with the first node, reset the cost and add to the closed vector
				PathNode* current = pOnNode;
				current->GetNodeData().cost = 0.0f;
				current->GetNodeData().isClosed = true;
				mClosedNodes.push_back(current);

				// While we have not found the goal node and we still have nodes to check - Main A* Logic
				while (current != goal && mOpenedNodes.size() >= 0)
				{
					#pragma region A* Logic

					// Loop through all neighbours of the current node (given the seeker properties)
					for (int i = 0; PathUtility::NeighbourIterFromInt(i, mIncludeDiagonal); i++)
					{
						PathNode* neighbourNode = nullptr;
						NEIGHBOUR neighbourType = static_cast<NEIGHBOUR>(i);

						// Attempt to get a non-nullptr neighbour
						if (current->GetNeighbour(neighbourType, neighbourNode))
						{
							// If an exclusive search is desired, then invoke upon neighbour
							bool excluded = hasExclude ? excludeSearch.operator()(neighbourNode) : true;

							// Does the neighbour meet requirements for search?
							if (neighbourNode->GetNodeData().isRouted && !neighbourNode->GetNodeData().isClosed && excluded)
							{
								// Get default search cost for additional step in path
								float cost = PathUtility::IsDiagonalNeighbour(neighbourType) ? mDiagCost : mStraightCost;

								// If a cost map has been provided then apply any additional costings
								if (hasCostMap)
								{
									float highestCost = 0.0f;

									// Search through the map and look for matches
									for (costIter = costSearch.begin(); costIter != costSearch.end(); costIter++)
									{
										if (!costIter->first.operator()(neighbourNode))
											continue;
										
										// Accumulate costs from the map of desired
										if (accumulateCostSearch)
										{
											cost += costIter->second;
										}
										else
										{
											// Otherwise store the highest cost (default)
											if (costIter->second)
												highestCost = costIter->second;
										}
									}

									// If it's not desired to accumulate cost, add the highest found cost match as default behaviour
									if (!accumulateCostSearch)
										cost += highestCost;
								}

								// If the node has not already been opened before
								if (!neighbourNode->GetNodeData().isOpened)
								{
									// Apply scoring, add to opened vector and set parent for path to follow
									ApplyScoring(current, neighbourNode, goal, cost);
									neighbourNode->GetNodeData().isOpened = true;
									neighbourNode->SetParent(current);
									mOpenedNodes.push_back(neighbourNode);
								}
								else
								{
									// Node has already been opened, recheck it's cost with the new cost against the neighbour
									if (current->GetNodeData().cost + cost < neighbourNode->GetNodeData().cost)
									{
										// If we have a lower cost then this is a better neighbour to choose for pathing, so re-apply scoring and parent
										neighbourNode->SetParent(current);
										ApplyScoring(current, neighbourNode, goal, cost);
									}
								}
							}
						}
					}

					// Attempt to get a pointer to the next node in the path (the one with the best heustic value)
					PathNode* next = nullptr;
					if (GetBestNode(next))
					{
						// If the closed vector does not contain the best node found then we can accept it
						if (std::find(mClosedNodes.begin(), mClosedNodes.end(), next) == mClosedNodes.end())
						{
							// Set the best node found to the appropriate vector containers for next step of searching
							next->GetNodeData().isOpened = false;
							std::remove(mOpenedNodes.begin(), mOpenedNodes.end(), next);
							next->GetNodeData().isClosed = true;
							mClosedNodes.push_back(next);

							// Current node becomes the next node - progression of path
							current = next;
							continue;
						}
					}

					#pragma endregion

					// Return out if number of unsuccessful cycles reaches a limit - cannot find path in time!
					if (++cycles > CYCLE_SAFE)
						return false;

				}

				// Certify we have found the goal node
				if (std::find(mClosedNodes.begin(), mClosedNodes.end(), goal) == mClosedNodes.end())
				{
					// Something went wrong and the goal node is in the closed list - cannot find path in time!
					ResetContainers();
					return false;
				}
				else
				{
					// We have completed the path to the goal, now fill the path vector
					FillPathVector(goal);

					// Pass all nodes from the path vector to the buffer
					AddPathToBuffer(pathBuffer);

					// Path Complete!
					return true;
				}
			}
		}
	}
}