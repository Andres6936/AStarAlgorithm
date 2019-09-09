////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "AStar.hpp" // See header for copyright and usage information

#include <iostream>
#include <cmath>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace std::chrono;

// Global data

// The world map

constexpr short MAP_WIDTH = 20;
constexpr short MAP_HEIGHT = 20;

constexpr int worldMap[MAP_WIDTH * MAP_HEIGHT ] =
{

//  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 00
                1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1,   // 01
                1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 02
                1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 03
                1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 04
                1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 05
                1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 06
                1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 07
                1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 08
                1, 9, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,   // 09
                1, 9, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 10
                1, 9, 9, 9, 9, 9, 1, 9, 1, 9, 1, 9, 9, 9, 9, 9, 1, 1, 1, 1,   // 11
                1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 12
                1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,   // 13
                1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,   // 14
                1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,   // 15
                1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,   // 16
                1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 1, 9, 9, 9, 9,   // 17
                1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,   // 18
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // 19

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

    bool GetSuccessors( AStar <SearchNode> *nAStar, SearchNode *nParentNode );
	float GetCost( SearchNode &successor );
	bool IsSameState( SearchNode &rhs );

	void PrintNodeInfo(); 


};

bool SearchNode::IsSameState( SearchNode &rhs )
{
	// same state in a maze search is simply when (x,y) are the same
    return ( x == rhs.x ) && ( y == rhs.y );
}

void SearchNode::PrintNodeInfo()
{
	cout << "Node position : (" << setw(2) << x << ", " << setw(2) << y << ")\n";
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. The distance is estimate with Manhattan distance.
float SearchNode::GoalDistanceEstimate( SearchNode &nodeGoal )
{
	return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool SearchNode::IsGoal( SearchNode &nodeGoal )
{
    return ( x == nodeGoal.x ) && ( y == nodeGoal.y );
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool SearchNode::GetSuccessors( AStar <SearchNode> *nAStar, SearchNode *nParentNode )
{

	int parentX = -1;
	int parentY = -1;

    if ( nParentNode )
    {
        parentX = nParentNode->x;
        parentY = nParentNode->y;
	}
	

	SearchNode NewNode;

	// push each possible move except allowing the search to go backwards

    if (( GetMap( x - 1, y ) < 9 ) && !( parentX == x - 1 && parentY == y ))
	{
		NewNode = SearchNode( x - 1, y );
        nAStar->AddSuccessor( NewNode );
	}

    if (( GetMap( x + 1, y ) < 9 ) && !( parentX == x + 1 && parentY == y ))
    {
        NewNode = SearchNode( x + 1, y );
        nAStar->AddSuccessor( NewNode );
    }

    if (( GetMap( x, y - 1 ) < 9 ) && !( parentX == x && parentY == y - 1 ))
	{
        NewNode = SearchNode( x, y - 1 );
        nAStar->AddSuccessor( NewNode );
    }

    if (( GetMap( x, y + 1 ) < 9 ) && !( parentX == x && parentY == y + 1 ))
	{
		NewNode = SearchNode( x, y + 1 );
        nAStar->AddSuccessor( NewNode );
	}	

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving
float SearchNode::GetCost( SearchNode &successor )
{
    return ( float ) GetMap( successor.x, successor.y );
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

    // Create a start state
    SearchNode nodeStart( 3, 5 );

    // Define the goal state
    SearchNode nodeEnd( 17, 15 );

    // Create an instance of the search class...
    // Set Start and goal states
    AStar <SearchNode> aStar( nodeStart, nodeEnd );

    aStar.ComputePath( );

    if ( aStar.GetSearchState( ) == SearchState::SUCCEEDED )
    {
        cout << "\nSearch found goal state\n\n";

        int steps = 0;

        while ( aStar.GetSizePath( ) > 0 )
        {
            Point2D point = aStar.Walk( );

            cout << "Node position : (" << setw( 2 ) << point.x << ", " << setw( 2 ) << point.y << ")\n";

            steps += 1;
        }

        cout << "\nSolution steps: " << steps << endl;
        cout << "Number of steps: " << aStar.GetNumberSteps( ) << endl;

        // Once you're done with the solution you can free the nodes up
        aStar.FreeSolutionNodes();
    }
    else if ( aStar.GetSearchState( ) == SearchState::FAILED )
    {
        cout << "\nSearch terminated. Did not find goal state\n";

    }

    // Display the number of loops the search went through
    // cout << "SearchSteps : " << SearchSteps << "\n";

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
