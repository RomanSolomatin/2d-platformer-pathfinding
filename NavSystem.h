#pragma once

#include <vector>

struct PathNode
{
	int x_coord, z_coord, index, type;
	int bez[2];
	TSharedPtr<PathNode> parent;
	float G; // cumulative distance
	float H; // heuristic (estimated) distance to goal
	TArray<unsigned int> directions;

	float GetF() { return G + H; } // F = G + H

	float GetH(TSharedPtr<PathNode> goal) // distance from current cell to target cell
	{
		// Manhattan (least accurate)
		//float x = FPlatformMath::Abs((float)(this->x_coord - goal->x_coord));
		//float z = FPlatformMath::Abs((float)(this->z_coord - goal->z_coord));
		//return x + z;

		// Pythagorean
		float x = FPlatformMath::Pow(this->x_coord - goal->x_coord, 2.0f);
		float z = FPlatformMath::Pow(this->z_coord - goal->z_coord, 2.0f);
		return FPlatformMath::Sqrt(x + z);

		// Just use F (most accurate but slowest)
		//return 0;
	}

	void SetCoords(int x, int z, int id)
	{
		x_coord = x;
		z_coord = z;
		index = id;
	}

	PathNode()
	{
	}
};

struct JumpInfo
{
	unsigned int index;
	int bez[2];
	float jump_cost;
	TArray<unsigned int> jump_path;

	JumpInfo()
	{
	}
};

struct NavPoint
{
	unsigned int x_coord, z_coord, nav_type, collision;
	TArray<unsigned int> link_run;
	TArray<unsigned int> link_fall;
	TArray<JumpInfo> link_jump;
	TArray<TArray<unsigned int>> jump_paths;

	NavPoint()
	{
		collision = 1;
	}
};

class NavSystem
{
public:
	NavSystem(void);
	~NavSystem(void);

	void BuildNavigation(int jump_height, int pawn_height, int world_width, int world_height, std::vector<uint8> collision_map);
	FVector FindPath(FVector start, FVector goal);
	TArray<TSharedPtr<PathNode>> GetPath();

	void DeleteAll();
	void DeleteNav();
	void DeletePath();

	// params
	unsigned int mapWidth = 0;
	unsigned int mapHeight = 0;
	unsigned int cellSize = 32;

	// example map for testing
	std::vector<uint8> map1 = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,0,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,0,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	UWorld *world;

private:
	// navmap building
	void DetectPlatforms(std::vector<uint8>& MapIn);
	void CreateRunLinks();
	void CreateFallLinks();
	void CreateJumpLinks(int jumpHeight);
	void CalculateJumpAtPoint(int height, int base);
	void AddJumpLink(int target, int base, int height, int offset, int horizontal, TArray<unsigned int> path);

	// pathfinding
	void SetStartAndGoal(int start_x, int start_z, int goal_x, int goal_z);
	void CheckPath();
	TSharedPtr<PathNode> GetNextNode();
	void AddNodeToOpenList(int x, int z, float newCost, TSharedPtr<PathNode> parent, TArray<unsigned int> path, int type, int bez[2]);

	TArray<NavPoint> navMap;
	TArray<unsigned int> platformsReached;
	unsigned int maxDropsAfterJump = 10;
	int verticalSize = 1;

	TSharedPtr<PathNode> startNode;
	TSharedPtr<PathNode> goalNode;
	TArray<TSharedPtr<PathNode>> openList;
	TArray<TSharedPtr<PathNode>> visitedList;
	TArray<TSharedPtr<PathNode>> pathNodesToGoal;

};