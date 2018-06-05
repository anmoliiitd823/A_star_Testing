#include "stlastar.h" // See header for copyright and usage information
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/OccupancyGrid.h"
#include <cmath>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"


#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
using namespace std;

//Size of the Map
const int MAP_WIDTH = 500;
const int MAP_HEIGHT = 500;

//Weight of the obstacle
const int OBSTACLE_COST = 9;

//Path to the matrix.txt created by matrixnode in obstacle matrix
const char* PATH_TO_MATRIX = "/home/anmol/Desktop/A_star_Testing/src/a_star/matrix.txt";

int world_map[ MAP_WIDTH * MAP_HEIGHT ];
std::vector<signed char> v((MAP_WIDTH)*(MAP_HEIGHT),0); // A vector which will be used to store and processing points for occupancy grid visualizationm,
//Occupancy Grid is read by rviz for simulations
nav_msgs::OccupancyGrid map123;
/*
    ## This represents a 2-D grid map, in which each cell represents the probability of occupancy.

    1)Header header 
    2)MapMetaData info      //{MetaData for the map}

    # The map data, in row-major order, starting with (0,0). Occupancy probabilities are in the range [0,100].
    # Unknown is -1.
    3)int8[] datag
*/
//********************************************************************************************************************

//Trajectory message to be published at trajec topic
visualization_msgs::Marker Trajectory; 

//Returns the cost at position (x,y)
int GetMap( int x, int y )
{
  if( x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT )
    return OBSTACLE_COST;  
  return world_map[(y*MAP_WIDTH)+x];
}

//Each coordinate is represented as an object of MapSearchNode for A* 
class MapSearchNode
{
public:
  int x;   // the (x,y) positions of the node
  int y;   
  MapSearchNode() { x = y = 0;  }
  MapSearchNode( int px, int py ) { x=px; y=py; }
  float GoalDistanceEstimate( MapSearchNode &nodeGoal );
  bool IsGoal( MapSearchNode &nodeGoal );
  bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
  float GetCost( MapSearchNode &successor );
  bool IsSameState( MapSearchNode &rhs );
  void PrintNodeInfo(); 
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

  // same state in a maze search is simply when (x,y) are the same
  if( (x == rhs.x) && (y == rhs.y) )
    return true;
  return false;
}

void MapSearchNode::PrintNodeInfo()
{
    // Adds all the required information of the trajectory node to trajectory msg and prints it at the console
    Trajectory.header.frame_id= "/map";
    Trajectory.header.stamp= ros::Time::now();
    Trajectory.ns= "Trajectory";
    Trajectory.action= visualization_msgs::Marker::ADD;
    Trajectory.pose.orientation.w= 1.0;
    Trajectory.id = 0;
    Trajectory.type = visualization_msgs::Marker::CUBE_LIST;
    Trajectory.scale.x = 0.5;
    Trajectory.scale.y = 0.5;
    Trajectory.scale.z = 0.0;
    Trajectory.color.g = 1.0;
    Trajectory.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    Trajectory.points.push_back(p);
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x,y );
    cout << str;
}
// Here's the heuristic function(Currently Based on Euclidean Distance) that estimates the distance from a Node
// to the Goal. 
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
  return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y); 
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

  if( (x == nodeGoal.x) && (y == nodeGoal.y) )
    return true;
  return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
  int parent_x = -1; 
  int parent_y = -1; 
  if( parent_node )
  {
    parent_x = parent_node->x;
    parent_y = parent_node->y;
  }

  MapSearchNode NewNode;

  // push each possible move except allowing the search to go backwards

  if( (GetMap( x-1, y ) < OBSTACLE_COST) && !((parent_x == x-1) && (parent_y == y))) 
  {
    NewNode = MapSearchNode( x-1, y );
    astarsearch->AddSuccessor( NewNode );
  } 
  if( (GetMap( x, y-1 ) < OBSTACLE_COST) && !((parent_x == x) && (parent_y == y-1))) 
  {
    NewNode = MapSearchNode( x, y-1 );
    astarsearch->AddSuccessor( NewNode );
  } 
  if( (GetMap( x+1, y ) < OBSTACLE_COST) && !((parent_x == x+1) && (parent_y == y))) 
  {
    NewNode = MapSearchNode( x+1, y );
    astarsearch->AddSuccessor( NewNode );
  } 
  if( (GetMap( x, y+1 ) < OBSTACLE_COST) && !((parent_x == x) && (parent_y == y+1)))
  {
    NewNode = MapSearchNode( x, y+1 );
    astarsearch->AddSuccessor( NewNode );
  }
  return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
  return (float) GetMap( x, y );

}


void matrixCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  cout<<"matrix received \n";
  // std::vector<int>::const::iterator it = array->;
    for(int i=0;i<MAP_WIDTH;i++){
    for(int j=0;j<MAP_HEIGHT;j++){
        int current = array->data[i+MAP_HEIGHT*j];
        world_map[i+MAP_HEIGHT*j] = current;
        v[i+MAP_HEIGHT*j] = current*20;
      }
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "a_star");
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1); 
  ros::Publisher trajec = n.advertise<visualization_msgs::Marker>("trajec", 10);
  ros::Rate r(30);       //Change rate to update the number of frames(maps) to be processed per second
  ros::Subscriber matrix_sub = n.subscribe("matrix", 1, matrixCallback);



  while (ros::ok())
  {
    ros::spinOnce();

  // Create an instance of the search class...
    AStarSearch<MapSearchNode> astarsearch;
    unsigned int SearchCount = 0;
    const unsigned int NumSearches = 1;
    while(SearchCount < NumSearches)
    {

        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.x = 250;
        nodeStart.y = 250; 
        //Make the start state obstacle free
        world_map[nodeStart.x+MAP_HEIGHT*nodeStart.y]=0;
    
        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.x = 0;           
        nodeEnd.y = 0; 
        //Make the end state obstacle free
        world_map[nodeEnd.x+MAP_HEIGHT*nodeEnd.y]=0;


        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do
        {
            SearchState = astarsearch.SearchStep();
            SearchSteps++;

            #if DEBUG_LISTS
                cout << "Steps:" << SearchSteps << "\n";
                int len = 0;
                cout << "Open:\n";
                MapSearchNode *p = astarsearch.GetOpenListStart();
                while( p )
                {
                  len++;
                  #if !DEBUG_LIST_LENGTHS_ONLY      
                    ((MapSearchNode *)p)->PrintNodeInfo();
                  #endif
                  p = astarsearch.GetOpenListNext();    
                }
                cout << "Open list has " << len << " nodes\n";
                len = 0;
                cout << "Closed:\n";
                p = astarsearch.GetClosedListStart();
                while( p )
                {
                    len++;
                    #if !DEBUG_LIST_LENGTHS_ONLY      
                        p->PrintNodeInfo();
                    #endif      
                    p = astarsearch.GetClosedListNext();
                }
                cout << "Closed list has " << len << " nodes\n";
            #endif
        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            cout << "Search found goal state\n";
            MapSearchNode *node = astarsearch.GetSolutionStart();
            #if DISPLAY_SOLUTION
                  cout << "Displaying solution\n";
            #endif
            int steps = 0;
            node->PrintNodeInfo();
            for( ;; )
            {
                node = astarsearch.GetSolutionNext();
                if( !node )
                    break;
                node->PrintNodeInfo();
                steps ++;
            };
            cout << "Solution steps " << steps << endl;
            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();
        }   
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
          cout << "Search terminated. Did not find goal state\n";        
        
        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";
        SearchCount ++;
        astarsearch.EnsureMemoryFreed();
    }
    map123.data = v;
    map123.info.resolution = 1;     //default =1                //resolution of map....m/cell | google "nav_msgs/MapMetaData" for more information
    map123.info.width = MAP_WIDTH;               //default m_size                  
    map123.info.height = MAP_HEIGHT;
    map_pub.publish(map123); 
    trajec.publish(Trajectory);
    //rviz_visual_tools::RvizVisualTools::deleteAllMarkers()
    r.sleep();
  }
}