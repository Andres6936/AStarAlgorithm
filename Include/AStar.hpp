/*
 * A* Algorithm Implementation using STL is
 * Copyright (C)2001-2005 Justin Heyes-Jones
 *
 * Permission is given by the author to freely redistribute and
 * include this code in any program as long as this credit is
 * given where due.
 *
 *   COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS,
 *   WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 *   INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED CODE
 *   IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
 *   OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND
 *   PERFORMANCE OF THE COVERED CODE IS WITH YOU. SHOULD ANY COVERED
 *   CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL
 *   DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY
 *   NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF
 *   WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE
 *   OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
 *   THIS DISCLAIMER.
 *
 *   Use at your own risk!
 *
 */

#ifndef STLASTAR_H
#define STLASTAR_H

// used for text debugging
#include <iostream>
#include <cstdio>

//#include <conio.h>
#include <cassert>

// stl includes
#include <algorithm>
#include <set>
#include <vector>
#include <queue>
#include <cfloat>

enum class SearchState : short
{
    NOT_INITIALISED,
    OUT_OF_MEMORY,
    SEARCHING,
    SUCCEEDED,
    FAILED
};

class Point2D
{

public:

    int x;
    int y;

    Point2D( )
    {
        x = 0;
        y = 0;
    }

    Point2D( int X, int Y )
    {
        x = X;
        y = Y;
    }
};

using namespace std;

/**
 * The main class is called AStar, and is a template class.
 * I chose to use templates because this enables the user to specialise
 * the AStarSearch class to their user state in an efficient way.
 *
 * Originally I used inheritence from a virtual base class, but
 * that lead to the use of type casts in many places to convert from
 * the base Node to the user's node. Also templates are resolved
 * at compile time rather than runtime and this makes them more
 * efficient and require less memory.
 *
 * The AStar search class. UserState is the users state space type
 */
template <class UserState> class AStar
{

public:

    /**
     * A node represents a possible state in the search
     * The user provided state type is included inside this type
     */
    class Node
    {

    public:

        Node *parent; // used during the search to record the parent of successor nodes
        Node *child; // used after the search for the application to view the search in reverse

        float g; // cost of this node + it's predecessors
        float h; // heuristic estimate of distance to goal
        float f; // sum of cumulative cost of predecessors and self and heuristic

        Node()
        {
            parent = nullptr;
            child = nullptr;
            g = 0.0f;
            h = 0.0f;
            f = 0.0f;
        }

        UserState m_UserState;
    };

private: // data

    // Heap (simple vector but used as a heap, cf. Steve Rabin's game gems article)
    // This is where we will remember which nodes we haven't yet expanded.
    vector< Node *> m_OpenList;

    // Closed list is a vector.
    // This is where we will remember which nodes we have expanded.
    vector< Node * > m_ClosedList;

    // Successors is a vector filled out by the user each type successors to a node
    // are generated
    vector< Node * > m_Successors;

    queue <Point2D> m_Points;

    // State
    SearchState m_State;

    // Counts steps
    int m_Steps;

    // Start and goal state pointers
    Node *m_Start;
    Node *m_Goal;

    Node *m_CurrentSolutionNode;

    // debugging : count memory allocation and free's
    int m_AllocateNodeCount;

public: // data

	// For sorting the heap the STL needs compare function that lets us compare
	// the f value of two nodes

    class HeapCompare_f
	{
		public:

			bool operator() ( const Node *x, const Node *y ) const
			{
				return x->f > y->f;
			}
	};


public: // methods


	// constructor just initialises private data
    AStar( )
    {
        m_State = SearchState::NOT_INITIALISED;
        m_AllocateNodeCount = 0;
        m_Steps = 0;
    }

    // Advances search
    SearchState ComputePath( UserState Start, UserState Goal )
    {
        m_CurrentSolutionNode = nullptr;
        m_AllocateNodeCount = 0;

        m_Start = new Node( );
        m_AllocateNodeCount += 1;

        m_Goal = new Node( );
        m_AllocateNodeCount += 1;

        assert(( m_Start != nullptr && m_Goal != nullptr ));

        m_Start->m_UserState = Start;
        m_Goal->m_UserState = Goal;

        m_State = SearchState::SEARCHING;

        // Initialise the AStar specific parts of the Start Node
        // The user only needs fill out the state information

        m_Start->g = 0;
        m_Start->h = m_Start->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
        m_Start->f = m_Start->g + m_Start->h;
        m_Start->parent = nullptr;

        // Push the start node on the Open list

        m_OpenList.push_back( m_Start ); // heap now unsorted

        // Initialise counter for search steps
        m_Steps = 0;

		// Firstly break if the user has not initialised the search
        assert( m_State != SearchState::NOT_INITIALISED );
        assert( m_State == SearchState::SEARCHING );

		// Next I want it to be safe to do a searchstep once the search has succeeded...
        if ( m_State == SearchState::SUCCEEDED || m_State == SearchState::FAILED )
		{
			return m_State; 
		}

        while ( m_State == SearchState::SEARCHING )
        {
            // Failure is defined as emptying the open list as there is nothing left to
            // search...
            // New: Allow user abort
            if ( m_OpenList.empty( ))
            {
                FreeAllNodes( );
                m_State = SearchState::FAILED;
                return m_State;
            }

            // Incremement step count
            m_Steps++;

            // Pop the best node (the one with the lowest f)
            Node *n = m_OpenList.front( ); // get pointer to the node
            pop_heap( m_OpenList.begin( ), m_OpenList.end( ), HeapCompare_f( ));
            m_OpenList.pop_back( );

            // Check for the goal, once we pop that we're done
            if ( n->m_UserState.IsGoal( m_Goal->m_UserState ))
            {
                // The user is going to use the Goal Node he passed in
                // so copy the parent pointer of n
                m_Goal->parent = n->parent;
                m_Goal->g = n->g;

                // A special case is that the goal was passed in as the start state
                // so handle that here
                if ( false == n->m_UserState.IsSameState( m_Start->m_UserState ))
                {
                    m_AllocateNodeCount--;
                    delete ( n );

                    // set the child pointers in each node (except Goal which has no child)
                    Node *nodeChild = m_Goal;
                    Node *nodeParent = m_Goal->parent;

                    do
                    {
                        nodeParent->child = nodeChild;

                        nodeChild = nodeParent;
                        nodeParent = nodeParent->parent;

                    }
                    while ( nodeChild != m_Start ); // Start is always the first node by definition

                }

                // delete nodes that aren't needed for the solution
                FreeUnusedNodes( );
                // Clear the list of points
                while ( !m_Points.empty( ))
                { m_Points.pop( ); }

                m_State = SearchState::SUCCEEDED;

                // Store the start point
                m_Points.push( Point2D( m_Start->m_UserState.x, m_Start->m_UserState.y ));

                m_CurrentSolutionNode = m_Start;

                while ( m_CurrentSolutionNode->child )
                {
                    Node *child = m_CurrentSolutionNode->child;

                    m_Points.push( Point2D( child->m_UserState.x, child->m_UserState.y ));

                    m_CurrentSolutionNode = m_CurrentSolutionNode->child;
                }

                return m_State;
            }
            else // not goal
            {

                // We now need to generate the successors of this node
                // The user helps us to do this, and we keep the new nodes in
                // m_Successors ...

                m_Successors.clear( ); // empty vector of successor nodes to n

                // User provides this functions and uses AddSuccessor to add each successor of
                // node 'n' to m_Successors
                bool ret;

                if ( n->parent )
                {
                    ret = n->m_UserState.GetSuccessors( this, &n->parent->m_UserState );
                }
                else
                {
                    ret = n->m_UserState.GetSuccessors( this, NULL );
                }

                if ( !ret )
                {

                    // free the nodes that may previously have been added
                    for ( AStar::Node *successor: m_Successors )
                    {
                        m_AllocateNodeCount -= 1;
                        delete successor;
                    }

                    m_Successors.clear( ); // empty vector of successor nodes to n

                    // free up everything else we allocated
                    m_AllocateNodeCount--;
                    delete (( n ));
                    FreeAllNodes( );

                    m_State = SearchState::OUT_OF_MEMORY;
                    return m_State;
                }

                // Now handle each successor to the current node ...
                for ( AStar::Node *successor: m_Successors )
                {

                    // 	The g value for this successor ...
                    float ValueGSuccessor = n->g + n->m_UserState.GetCost( successor->m_UserState );

                    // Now we need to find whether the node is on the open or closed lists
                    // If it is but the node that is already on them is better (lower g)
                    // then we can forget about this successor

                    // First linear search of open list to find node

                    typename vector <Node *>::iterator openlist_result;

                    for ( openlist_result = m_OpenList.begin( );
                          openlist_result != m_OpenList.end( ); openlist_result++ )
                    {
                        if (( *openlist_result )->m_UserState.IsSameState( successor->m_UserState ))
                        {
                            break;
                        }
                    }

                    if ( openlist_result != m_OpenList.end( ))
                    {

                        // we found this state on open

                        if (( *openlist_result )->g <= ValueGSuccessor )
                        {
                            m_AllocateNodeCount--;
                            delete (( successor ));

                            // the one on Open is cheaper than this one
                            continue;
                        }
                    }

                    typename vector <Node *>::iterator closedlist_result;

                    for ( closedlist_result = m_ClosedList.begin( );
                          closedlist_result != m_ClosedList.end( ); closedlist_result++ )
                    {
                        if (( *closedlist_result )->m_UserState.IsSameState(( successor )->m_UserState ))
                        {
                            break;
                        }
                    }

                    if ( closedlist_result != m_ClosedList.end( ))
                    {

                        // we found this state on closed

                        if (( *closedlist_result )->g <= ValueGSuccessor )
                        {
                            // the one on Closed is cheaper than this one
                            m_AllocateNodeCount--;
                            delete (( successor ));

                            continue;
                        }
                    }

                    // This node is the best node so far with this particular state
                    // so lets keep it and set up its AStar specific data ...

                    successor->parent = n;
                    successor->g = ValueGSuccessor;
                    successor->h = successor->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
                    successor->f = successor->g + successor->h;

                    // Successor in closed list
                    // 1 - Update old version of this node in closed list
                    // 2 - Move it from closed to open list
                    // 3 - Sort heap again in open list

                    if ( closedlist_result != m_ClosedList.end( ))
                    {
                        // Update closed node with successor node AStar data
                        //*(*closedlist_result) = *(*successor);
                        ( *closedlist_result )->parent = successor->parent;
                        ( *closedlist_result )->g = successor->g;
                        ( *closedlist_result )->h = successor->h;
                        ( *closedlist_result )->f = successor->f;

                        // Free successor node
                        m_AllocateNodeCount--;
                        delete ( successor );

                        // Push closed node into open list
                        m_OpenList.push_back(( *closedlist_result ));

                        // Remove closed node from closed list
                        m_ClosedList.erase( closedlist_result );

                        // Sort back element into heap
                        push_heap( m_OpenList.begin( ), m_OpenList.end( ), HeapCompare_f( ));

                        // Fix thanks to ...
                        // Greg Douglas <gregdouglasmail@gmail.com>
                        // who noticed that this code path was incorrect
                        // Here we have found a new state which is already CLOSED

                    }

                        // Successor in open list
                        // 1 - Update old version of this node in open list
                        // 2 - sort heap again in open list

                    else if ( openlist_result != m_OpenList.end( ))
                    {
                        // Update open node with successor node AStar data
                        //*(*openlist_result) = *(*successor);
                        ( *openlist_result )->parent = successor->parent;
                        ( *openlist_result )->g = successor->g;
                        ( *openlist_result )->h = successor->h;
                        ( *openlist_result )->f = successor->f;

                        // Free successor node
                        m_AllocateNodeCount--;
                        delete ( successor );

                        // re-make the heap
                        // make_heap rather than sort_heap is an essential bug fix
                        // thanks to Mike Ryynanen for pointing this out and then explaining
                        // it in detail. sort_heap called on an invalid heap does not work
                        make_heap( m_OpenList.begin( ), m_OpenList.end( ), HeapCompare_f( ));
                    }

                        // New successor
                        // 1 - Move it from successors to open list
                        // 2 - sort heap again in open list

                    else
                    {
                        // Push successor node into open list
                        m_OpenList.push_back(( successor ));

                        // Sort back element into heap
                        push_heap( m_OpenList.begin( ), m_OpenList.end( ), HeapCompare_f( ));
                    }

                }

                // push n onto Closed, as we have expanded it now

                m_ClosedList.push_back( n );

            } // end else (not goal so expand)

        }

        return m_State; // Succeeded bool is false at this point.
    }

	// User calls this to add a successor to a list of successors
	// when expanding the search frontier
	bool AddSuccessor( UserState &State )
	{
		Node *node = new Node();
		m_AllocateNodeCount += 1;

        node->m_UserState = State;
        m_Successors.push_back( node );
        return true;
    }

	// Free the solution nodes
	// This is done to clean up all used Node memory when you are done with the
	// search
	void FreeSolutionNodes()
	{
		Node *n = m_Start;

		if( m_Start->child )
		{
			do
			{
				Node *del = n;
				n = n->child;

                m_AllocateNodeCount--;
                delete ( del );

            }
            while ( n != m_Goal );

            m_AllocateNodeCount--;
            delete ( n ); // Delete the goal

		}
		else
		{
			// if the start node is the solution we need to just delete the start and goal
			// nodes
            m_AllocateNodeCount--;
            delete ( m_Start );

            m_AllocateNodeCount--;
            delete ( m_Goal );
        }

	}

    SearchState GetSearchState( )
    { return m_State; }

    unsigned int GetNumberSteps( )
    { return m_Steps; }

	// Functions for traversing the solution

    Point2D Walk( )
    {
        Point2D point = m_Points.front( );
        m_Points.pop( );

        return point;
    }

    unsigned int GetSizePath( )
    {
        return m_Points.size( );
    }

    // Get end node
	UserState *GetSolutionEnd()
	{
		m_CurrentSolutionNode = m_Goal;
		if( m_Goal )
		{
			return &m_Goal->m_UserState;
		}
		else
		{
			return NULL;
		}
	}
	
	// Step solution iterator backwards
	UserState *GetSolutionPrev()
	{
		if( m_CurrentSolutionNode )
		{
			if( m_CurrentSolutionNode->parent )
			{

				Node *parent = m_CurrentSolutionNode->parent;

				m_CurrentSolutionNode = m_CurrentSolutionNode->parent;

				return &parent->m_UserState;
			}
		}

		return NULL;
	}

private: // methods

	// This is called when a search fails or is cancelled to free all used
	// memory 
	void FreeAllNodes()
	{
		// iterate open list and delete all nodes
        for ( AStar::Node *iterOpen: m_OpenList )
        {
            m_AllocateNodeCount -= 1;
            delete iterOpen;
		}

		m_OpenList.clear();

		// iterate closed list and delete unused nodes
        for ( AStar::Node *iterClosed: m_ClosedList )
        {
            m_AllocateNodeCount -= 1;
            delete iterClosed;
		}

		m_ClosedList.clear();

		// delete the goal
        m_AllocateNodeCount -= 1;
        delete m_Goal;
	}


	// This call is made by the search class when the search ends. A lot of nodes may be
	// created that are still present when the search ends. They will be deleted by this 
	// routine once the search ends
	void FreeUnusedNodes()
	{
		// iterate open list and delete unused nodes
		for( AStar::Node *iterOpen: m_OpenList)
		{
			if( !iterOpen->child )
			{
				m_AllocateNodeCount -= 1;
				delete iterOpen;
			}
		}

		m_OpenList.clear();

		// iterate closed list and delete unused nodes
		for( AStar::Node *iterClosed: m_ClosedList )
		{
			if( !iterClosed->child )
			{
                m_AllocateNodeCount -= 1;
                delete iterClosed;
			}
		}

		m_ClosedList.clear();
	}
};

#endif