#include "NavSystem.h"

/****************************************************************************************************

	Pathfinding system for 2D sidescrolling platformer pawn.
	
	An instance is created within the pawn each time it needs to calculate a new path if the terrain 
	has changed since the last path calculation, or the jump height of the pawn changes. Each pawn 
	needs its own instance, as they have different jump heights etc.

 ****************************************************************************************************/

NavSystem::NavSystem(void)
{
	navMap.Empty();
}

NavSystem::~NavSystem(void)
{
}

TArray<TSharedPtr<PathNode>> NavSystem::GetPath()
{
	return pathNodesToGoal;
}

// Initialize properties and populate node graph
void NavSystem::BuildNavigation(int jump_height, int pawn_height, int world_width, int world_height, std::vector<uint8> collision_map)
{
	mapWidth = world_width;
	mapHeight = world_height;
	verticalSize = pawn_height;
	DetectPlatforms(collision_map);
	CreateRunLinks();
	CreateFallLinks();
	CreateJumpLinks(jump_height);
}

// Create a node graph describing each possible location the pawn could stand,
// and determine whether it's at the edge or in the middle
void NavSystem::DetectPlatforms(std::vector<uint8>& MapIn)
{
	// 0 = no nav point
	// 1 = platform left edge
	// 2 = platform middle
	// 3 = platform right edge
	// 4 = lone platform

	navMap.Empty();
	navMap.AddZeroed(mapWidth * mapHeight);

	int platformIndex = 0;
	bool bPlatformStarted = false;

	for (size_t z = 0; z < mapHeight; z++)
	{
		bPlatformStarted = false; // reset when starting new row

		for (size_t x = 0; x < mapWidth; x++)
		{
			int index = z * mapWidth + x;
			if (index >= navMap.Num()) { return; } // out of bounds check
			
			navMap[index].x_coord = x;
			navMap[index].z_coord = z;
			navMap[index].collision = MapIn[index];

			if (!bPlatformStarted)
			{
				if (MapIn[index] == 1 && MapIn[index - mapWidth] == 0) // if target tile is free and one below has collision, can stand there
				{
					navMap[index].nav_type = 1; // the platform hadn't started yet, so it must be a left edge (or lone platform)
					bPlatformStarted = true;
					platformIndex++;
				}
			}

			if (bPlatformStarted)
			{
				if (MapIn[index - mapHeight + 1] == 0	// if lower right tile collides 
					&& MapIn[index + 1] == 1			// and right tile doesn't
					&& navMap[index].nav_type != 1)		// and it's not a left edge
				{
					navMap[index].nav_type = 2;			// then it's a middle navpoint
				}

				if (MapIn[index - mapHeight + 1] == 1	// if lower right tile is free 
					|| MapIn[index + 1] == 0)			// or right tile collides
				{
					if (navMap[index].nav_type == 1)	// and navpoint is a left edge
					{
						navMap[index].nav_type = 4;		// it's a lone platform
					}
					else
					{
						navMap[index].nav_type = 3;		// otherwise it's a right edge
					}

					bPlatformStarted = false;
				}
			}
		}
	}
}

void NavSystem::CreateRunLinks()
{
	int mapSize = mapWidth * mapHeight;
	for (size_t i = 0; i < mapSize; i++)
	{
		// not at the extreme right side
		if (navMap[i].nav_type != 0 && (i + 1) % mapWidth != 0)
		{
			if (navMap[i + 1].nav_type != 0)
			{
				navMap[i].link_run.Add(i + 1); // add a new floor link from target navpoint to next right navpoint
				navMap[i + 1].link_run.Add(i); // and in reverse, so you can go left
			}
		}
	}
}

void NavSystem::CreateFallLinks()
{
	int mapSize = mapWidth * mapHeight;
	int a = 0, b = 0;
	int sideTile, targetRow, checkNavPoint;
	for (size_t i = 0; i < mapSize; i++)
	{
		if (navMap[i].nav_type == 1 || navMap[i].nav_type == 3 || navMap[i].nav_type == 4) // left edge / right edge / lone
		{
			switch (navMap[i].nav_type)
			{
			case 3: //right edge
				a = 1;
				b = 1;
				break;

			case 1: //left edge
				a = 0;
				b = 0;
				break;

			case 4: // lone
				a = 0;
				b = 1;
				break;

			default:
				break;
			}

			for (size_t j = a; j <= b; j++)
			{
				if (j == 0)
				{
					sideTile = i - 1; //sideTile = next left tile
				}
				else
				{
					sideTile = i + 1; //sideTile = next right tile
				}
				
				if (navMap[sideTile].collision == 1)
				{
					targetRow = navMap[sideTile].z_coord - 1;

					while (targetRow > 0)
					{
						checkNavPoint = targetRow * mapWidth + navMap[sideTile].x_coord;

						if (navMap[checkNavPoint].nav_type != 0)
						{
							navMap[i].link_fall.Add(checkNavPoint); //add a new fall link from target navpoint to navPointToCheck
							break;
						}

						targetRow--;
					}
				}
			}
		}
	}
}

void NavSystem::CreateJumpLinks(int jumpHeight)
{
	int mapSize = mapWidth * mapHeight;
	for (size_t i = 0; i < mapSize; i++)
	{
		if (navMap[i].nav_type != 0)
		{
			platformsReached.Empty();
			for (size_t j = 1; j <= jumpHeight; j++)
			{
				CalculateJumpAtPoint(j, i);
			}
		}
	}
}

void NavSystem::CalculateJumpAtPoint(int height, int base)
{
	if (base >= navMap.Num()) { return; }
	
	int x = navMap[base].x_coord;
	int z = navMap[base].z_coord;
	int index = 0, horizontal;
	bool bLeft;
	TArray<unsigned int> path;
	bool bSkip;

	for (int i = 0; i <= 1; i++) // left and right
	{
		bLeft = false;
		if (i == 0) bLeft = true;

		for (int offset = height - 1; offset >= 0; offset--)
		{
			bSkip = false;
			path.Empty();
			path.Add(base);

			for (size_t f = 1; f <= offset; f++) // go up til offset height - 1
			{
				index = (z + f) * mapWidth + x;

				for (size_t i = 0; i <= verticalSize; i++)
				{
					if (index + i * (int)mapWidth < navMap.Num())
					{
						if (navMap[index + i * mapWidth].collision == 0)
						{
							bSkip = true;
							break;
						}
					}
				}
				
				if (bSkip) break;
				
				path.Add(index);
			}

			horizontal = 1;
			index = (z + 1 + offset) * mapWidth + x;

			// if tile above collides then is not valid
			for (size_t i = 0; i <= verticalSize; i++)
			{
				if (index + i * (int)mapWidth < navMap.Num())
				{
					if (navMap[index + i * mapWidth].collision == 0)
					{
						bSkip = true;
						break;
					}
				}
			}

			if (!bSkip)
			{
				for (size_t j = 1 + offset; j <= height; j++) // go up til jump height
				{
					index = (z + j) * mapWidth + x + horizontal * (bLeft ? 1 : -1);
					if (navMap[index].collision == 0) // if collides then not a valid jump
					{
						bSkip = true;
						break;
					}

					// or if tile above collides
					for (size_t i = 0; i <= verticalSize; i++)
					{
						if (index + i * (int)mapWidth < navMap.Num())
						{
							if (navMap[index + i * mapWidth].collision == 0)
							{
								bSkip = true;
								break;
							}
						}
					}

					path.Add(index);

					if (navMap[index].nav_type != 0)
					{
						AddJumpLink(index, base, height, offset, horizontal, path);
						bSkip = true;
						break;
					}
					
					horizontal++;
				}
			}

			if (!bSkip)
			{
				for (size_t j = 1; j <= height; j++) // go back down til level with jump start height
				{
					index = (z + height - j) * mapWidth + x + horizontal * (bLeft ? 1 : -1);
					if (navMap[index].collision == 0)
					{
						bSkip = true;
						break;
					} // if collides then not a valid jump
					path.Add(index);
					
					if (navMap[index].nav_type != 0) // if row below is a platform, is a valid landing point
					{
						AddJumpLink(index, base, height, offset, horizontal, path);
						bSkip = true;
						break;
					}

					if (j < height - offset)
					{
						horizontal++;
					}
				}
			}

			if (!bSkip)
			{
				int check = 0;
				for (size_t j = 1; j <= maxDropsAfterJump; j++)
				{
					check = (z - j) * mapWidth + x + horizontal  * (bLeft ? 1 : -1);
					
					if (check <= 1 || navMap[check].collision == 0) break;
					path.Add(check);
					
					if (navMap[check].nav_type != 0)
					{
						// add jump link
						AddJumpLink(check, base, height + j, offset, horizontal, path);
						break;
					}
				}
			}
		}
	}
}

void NavSystem::AddJumpLink(int target, int base, int height, int offset, int horizontal, TArray<unsigned int> path)
{
	if (!platformsReached.Contains(target))
	{
		platformsReached.Add(target);
		TSharedRef<JumpInfo> newJump(new JumpInfo());
		newJump->index = target;

		int z = 0, highest = 0, z_index = -1;
		for (size_t i = 0; i < path.Num(); i++)
		{
			z = FPlatformMath::FloorToInt(path[i] / mapWidth);
			if (z > highest)
			{
				highest = z;
				z_index = i;
			}
		}

		newJump->bez[0] = -1;
		newJump->bez[1] = -1;

		if (z_index >= 0)
		{
			int newZ = highest * mapWidth;
			newJump->bez[0] = newZ + base % mapWidth;
			newJump->bez[1] = newZ + target % mapWidth;
		}

		if (path.Num() > 0)
		{
			newJump->jump_path = path;
			navMap[base].jump_paths.Add(path);
		}

		newJump->jump_cost = FPlatformMath::Sqrt(FPlatformMath::Pow(horizontal, 2.0f) + FPlatformMath::Pow(height, 2.0f));
		navMap[base].link_jump.Add(*newJump);

		//UE_LOG(LogTemp, Error, TEXT("add jump from %d to %d (height: %d, offset %d, across %d)"), base, target, height, offset, horizontal * LR);
	}
}

void NavSystem::SetStartAndGoal(int start_x, int start_z, int goal_x, int goal_z)
{
	int mapSize = mapWidth * mapHeight;
	int startIndex = start_z * mapWidth + start_x;
	int goalIndex = goal_z * mapWidth + goal_x;

	if (start_x >= 0 && start_z >= 0 && goal_x >= 0 && goal_z >= 0
		&& startIndex < mapSize && goalIndex < mapSize)
	{
		startNode = TSharedPtr<PathNode>(new PathNode());
		startNode->SetCoords(start_x, start_z, startIndex);

		goalNode = TSharedPtr<PathNode>(new PathNode());
		goalNode->SetCoords(goal_x, goal_z, goalIndex);

		startNode->G = 0.0f; // costs 0 to get to start from start
		startNode->H = startNode->GetH(goalNode); // estimated cost to get from start to end
		startNode->parent = 0; // first cell has no parent

		openList.Add(startNode); // add start cell to end of openList
		
		//UE_LOG(LogTemp, Error, TEXT("Set start (%d, %d) and goal (%d, %d)"), start_x, start_z, goal_x, goal_z);
	}
}

void NavSystem::CheckPath()
{
	if (openList.Num() == 0) // if nothing is in openList a path cannot be found
	{
		UE_LOG(LogTemp, Error, TEXT("No path to goal found."));
		return;
	}

	int index = 0;
	float fall = 0.0f;
	TSharedPtr<PathNode> currentNode = GetNextNode();

	if (currentNode->index == goalNode->index) // if goal reached
	{
		//UE_LOG(LogTemp, Error, TEXT("Goal found!"));
		
		// move backwards from goal finding shortest path back to start
		TSharedPtr<PathNode> getPath;
		for (getPath = currentNode; getPath.IsValid(); getPath = getPath->parent)
		{
			pathNodesToGoal.Add(getPath);
		}

		return;
	}
	else // if not reached goal
	{
		//UE_LOG(LogTemp, Error, TEXT("Not reached goal yet"));

		TArray<unsigned int> path;

		// run links
		for (size_t i = 0; i < navMap[currentNode->index].link_run.Num(); i++)
		{
			path.Add(currentNode->index);
			index = navMap[currentNode->index].link_run[i];

			if (navMap[index].x_coord > navMap[currentNode->index].x_coord) // if node goes to right
			{
				path.Add(currentNode->index + 1);
			}
			else if (navMap[index].x_coord < navMap[currentNode->index].x_coord) // else it goes left
			{
				path.Add(currentNode->index - 1);
			}
			int bezier[2] = { -1, -1 }; // no bezier needed for run
			AddNodeToOpenList(navMap[index].x_coord, navMap[index].z_coord, currentNode->G + 1.0f, currentNode, path, 1, bezier);
			path.Empty();
		}

		// fall links
		path.Empty();
		for (size_t i = 0; i < navMap[currentNode->index].link_fall.Num(); i++)
		{
			path.Add(currentNode->index);
			index = navMap[currentNode->index].link_fall[i];
			fall = 1.0f;
			if (navMap[currentNode->index].z_coord > navMap[index].z_coord)
			{
				fall = FPlatformMath::Sqrt(1.0f + FPlatformMath::Pow(navMap[currentNode->index].z_coord - navMap[index].z_coord, 2.0f));
			}
			
			int offset = 0;
			if (navMap[index].x_coord > navMap[currentNode->index].x_coord) // if node goes to right first
			{
				offset = 1;
			}
			else if (navMap[index].x_coord < navMap[currentNode->index].x_coord) // else it goes left
			{
				offset = -1;
			}
			path.Add(currentNode->index + offset);

			path.Add(navMap[index].z_coord * mapWidth + navMap[currentNode->index].x_coord + offset);

			int bezier[2] = { -1, -1 }; // no bezier needed for fall
			AddNodeToOpenList(navMap[index].x_coord, navMap[index].z_coord, currentNode->G + fall, currentNode, path, 2, bezier);
			path.Empty();
		}

		// jump links
		for (size_t i = 0; i < navMap[currentNode->index].link_jump.Num(); i++)
		{
			path.Empty();
			index = navMap[currentNode->index].link_jump[i].index;
			path.Add(index);
			AddNodeToOpenList(navMap[index].x_coord, navMap[index].z_coord, currentNode->G + navMap[currentNode->index].link_jump[i].jump_cost, 
				currentNode, path, 3, navMap[currentNode->index].link_jump[i].bez);
		}

		for (size_t i = 0; i < openList.Num(); i++) // remove from openList
		{
			if (currentNode->index == openList[i]->index)
			{
				openList.RemoveAt(i, 1);
			}
		}

		CheckPath();
	}
}

TSharedPtr<PathNode> NavSystem::GetNextNode() // finds next available node in openList with the lowest F value
{
	float bestF = 0.0f;
	int index = 0;
	TSharedPtr<PathNode> nextNode;

	for (size_t i = 0; i < openList.Num(); i++)
	{
		if (openList[i]->GetF() < bestF || i == 0)
		{
			bestF = openList[i]->GetF();
			index = i;
		}
	}

	if (index >= 0)
	{
		nextNode = openList[index];

		// add to visitedList, and remove from openList
		visitedList.Add(nextNode);
		openList.RemoveAt(index);
	}

	return nextNode;
}

void NavSystem::AddNodeToOpenList(int x, int z, float newCost, TSharedPtr<PathNode> parent, TArray<unsigned int> path, int type, int bez[2])
{
	// 1 = run
	// 2 = fall
	// 3 = jump

	int index = z * mapWidth + x;

	// check this cell id against visited list to make sure it's not already been checked
	for (size_t i = 0; i < visitedList.Num(); i++)
	{
		if (index == visitedList[i]->index)
		{
			return;
		}
	}

	// create new path search node
	TSharedPtr<PathNode> newChild = TSharedPtr<PathNode>(new PathNode());
	newChild->SetCoords(x, z, index);
	newChild->parent = parent;
	newChild->G = newCost;
	newChild->H = parent->GetH(goalNode);
	newChild->directions = path;
	newChild->type = type;
	newChild->bez[0] = bez[0];
	newChild->bez[1] = bez[1];

	for (size_t i = 0; i < openList.Num(); i++)
	{
		if (index == openList[i]->index)
		{
			float newF = newChild->G + openList[i]->H;

			if (newF < openList[i]->GetF()) // if new F is smaller than current F, replace it
			{
				openList[i]->G = newChild->G;
				openList[i]->parent = parent;
				openList[i]->directions = newChild->directions;
				openList[i]->type = newChild->type;
				openList[i]->bez[0] = newChild->bez[0];
				openList[i]->bez[1] = newChild->bez[1];
				openList[i]->SetCoords(newChild->x_coord, newChild->z_coord, newChild->index);
			}
			else return; // if new F is not smaller, ignore it
		}
	}

	openList.Add(newChild);
}

FVector NavSystem::FindPath(FVector start, FVector goal)
{
	// Remove old
	DeletePath();

	// Initialize start
	int start_x = FPlatformMath::FloorToInt(start.X / cellSize);
	int start_z = FPlatformMath::FloorToInt(start.Z / cellSize) - 1;
	int start_index = start_z * mapWidth + start_x;

	// Initialize goal
	int goal_x = FPlatformMath::FloorToInt(goal.X / cellSize);
	int goal_z = FPlatformMath::FloorToInt(goal.Z / cellSize);
	int goal_index = goal_z * mapWidth + goal_x;

	int mapSize = mapWidth * mapHeight;

	if (start_z < 0 || goal_z < 0 || start_index >= navMap.Num() || goal_index >= navMap.Num())
	{
		return FVector::ZeroVector;
	}

	if (navMap[start_index].nav_type == 0) // if start colliding, find nearest available nav point above (max 1 off)
	{
		if (start_index + (int)mapWidth < navMap.Num())
		{
			if (navMap[start_index + mapWidth].nav_type != 0)
			{
				start_z++;
				start_index += mapWidth;
			}
		}
	}

	bool bSkip = false;
	if (navMap[goal_index].nav_type == 0)
	{
		if (goal_index + (int)mapWidth < navMap.Num()) // if goal colliding, find nearest available nav point above (max 1 off)
		{
			if (navMap[goal_index + mapWidth].nav_type != 0)
			{
				goal_z++;
				goal_index += mapWidth;
				bSkip = true;
			}
		}

		if (!bSkip) // or below
		{
			int check = 0;
			for (int i = goal_z - 1; i > 0; i--)
			{
				check = i * mapWidth + goal_x;
				if (check < navMap.Num() && check >= 0)
				{
					if (navMap[check].nav_type != 0)
					{
						goal_z = i;
						goal_index = check;
						break;
					}
				}
			}
		}
	}

	// If start or goal is colliding then can't return the location
	if (navMap[start_z * mapWidth + start_x].collision == 0 || navMap[goal_z * mapWidth + goal_x].collision == 0)
	{
		return FVector::ZeroVector;
	}
	
	SetStartAndGoal(start_x, start_z, goal_x, goal_z); // initialise start and goal points
	CheckPath(); // begin pathfinding
	return FVector(goal_x * cellSize + (cellSize / 2), 32.0f, (goal_z + 1) * cellSize); // return world location for start and goal
}

void NavSystem::DeleteAll()
{
	DeleteNav();
	DeletePath();
}

void NavSystem::DeleteNav()
{
	navMap.Empty();
}

void NavSystem::DeletePath()
{
	startNode.Reset();
	goalNode.Reset();
	openList.Empty();
	visitedList.Empty();
	pathNodesToGoal.Empty();
}
