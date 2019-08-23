////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "AStar.hpp" // See header for copyright and usage information

#include <iostream>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <iomanip>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;
using namespace std::chrono;

// Global data

// The world map

constexpr short MAP_WIDTH = 20;
constexpr short MAP_HEIGHT = 20;

constexpr int worldMap[MAP_WIDTH * MAP_HEIGHT ] =
{

// 0001020304050607080910111213141516171819
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
	1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
	1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
	1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
	1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
	1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
	1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

};

// map helper functions

int GetMap( int x, int y )
{
	if( x < 0 || x >= MAP_WIDTH ||
	    y < 0 || y >= MAP_HEIGHT )
	{
		return 9;	 
	}

	return worldMap[ ( y * MAP_WIDTH) + x];
}



// Definitions

class SearchNode
{

public:

	int x;	 // the (x,y) positions of the node
	int y;	
	
	SearchNode() { x = y = 0; }
	SearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( SearchNode &nodeGoal );
	bool IsGoal( SearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<SearchNode> *astarsearch, SearchNode *parent_node );
	float GetCost( SearchNode &successor );
	bool IsSameState( SearchNode &rhs );

	void PrintNodeInfo(); 


};

bool SearchNode::IsSameState( SearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
    return ( x == rhs.x ) &&
           ( y == rhs.y );

}

void SearchNode::PrintNodeInfo()
{
	cout << "Node position : (" << setw(2) << x << ", " << setw(2) << y << ")\n";
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float SearchNode::GoalDistanceEstimate( SearchNode &nodeGoal )
{
	return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool SearchNode::IsGoal( SearchNode &nodeGoal )
{

    return ( x == nodeGoal.x ) &&
           ( y == nodeGoal.y );

}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool SearchNode::GetSuccessors( AStarSearch<SearchNode> *astarsearch, SearchNode *parent_node )
{

	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	

	SearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9) && !((parent_x == x-1) && (parent_y == y))
	  ) 
	{
		NewNode = SearchNode( x - 1, y );
		astarsearch->AddSuccessor( NewNode );
	}

    if( (GetMap( x+1, y ) < 9) && !((parent_x == x+1) && (parent_y == y))
            )
    {
        NewNode = SearchNode( x + 1, y );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x, y-1 ) < 9) && !((parent_x == x) && (parent_y == y-1))
	  ) 
	{
        NewNode = SearchNode( x, y - 1 );
        astarsearch->AddSuccessor( NewNode );
    }
		
	if( (GetMap( x, y+1 ) < 9) && !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = SearchNode( x, y + 1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float SearchNode::GetCost( SearchNode &successor )
{
	return (float) GetMap( x, y );

}


// Main

int main( int argc, char *argv[] )
{
    cout << "\nSTL A* Search implementation\n\n(C) 2001 Justin Heyes-Jones\n";

    // Use auto keyword to avoid typing long
    // type definitions to get the timepoint
    // at this instant use function now()
    auto start = high_resolution_clock::now();

	// Our sample problem defines the world as a 2d array representing a terrain
	// Each element contains an integer from 0 to 5 which indicates the cost 
	// of travel across the terrain. Zero means the least possible difficulty 
	// in travelling (think ice rink if you can skate) whilst 5 represents the 
	// most difficult. 9 indicates that we cannot pass.

	// Create an instance of the search class...

	AStarSearch<SearchNode> astarsearch;

	unsigned int SearchCount = 0;

    constexpr unsigned int NumSearches = 1;

	while(SearchCount < NumSearches)
	{

		// Create a start state
		SearchNode nodeStart;
		nodeStart.x = rand()%MAP_WIDTH;
		nodeStart.y = rand()%MAP_HEIGHT; 

		// Define the goal state
		SearchNode nodeEnd;
		nodeEnd.x = rand()%MAP_WIDTH;						
		nodeEnd.y = rand()%MAP_HEIGHT; 
		
		// Set Start and goal states
		
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

        SearchState = astarsearch.SearchStep();

        SearchSteps++;

		while( SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SEARCHING )
        {
            SearchState = astarsearch.SearchStep();

            SearchSteps++;
        }

		if( SearchState == AStarSearch<SearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "\nSearch found goal state\n\n";

				SearchNode *node = astarsearch.GetSolutionStart();

				int steps = 0;

				node->PrintNodeInfo();

				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					node->PrintNodeInfo();
					steps ++;
				
				};

				cout << "\nSolution steps " << steps << endl;

				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();

	
		}
		else if( SearchState == AStarSearch<SearchNode>::SEARCH_STATE_FAILED )
		{
			cout << "\nSearch terminated. Did not find goal state\n";
		
		}

		// Display the number of loops the search went through
		cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount ++;

		astarsearch.EnsureMemoryFreed();
	}

    // After function call
    auto stop = high_resolution_clock::now();


    // Subtract stop and start timepoints and
    // cast it to required unit. Predefined units
    // are nanoseconds, microseconds, milliseconds,
    // seconds, minutes, hours. Use duration_cast()
    // function.
    auto duration = duration_cast<microseconds>(stop - start);

    // To get the value of duration use the count()
    // member function on the duration object
    cout << "\nMicroseconds: " << duration.count() << "\n";

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
