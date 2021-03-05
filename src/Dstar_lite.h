// Including general libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <set>
#include <pluginlib/class_list_macros.h>

// Including ROS specific libraries
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// To accomodate for moving base
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <angles/angles.h>

// To accomodate sensory input
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

// Navigation messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

// Costmap transform
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// To get costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// Defining the whole thing in a namespace
using namespace std;
using std::string;

#ifndef PathPlanners_ROS
#define PathPlanners_ROS

// Structure to store the cells in a data type
struct cells{
    int currentCell;
    float fCost;
};  

struct node{
    int cell_index;
    float g_value;
    float rhs;
};

namespace PathPlanners_all{
  
class PathPlannersROS : public nav_core::BaseGlobalPlanner {
public:
    PathPlannersROS();
    PathPlannersROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Overriden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
    
    // Base variables for Planners
    float originX;float originY;float resolution;
    double step_size_, min_dist_from_robot_;
    bool initialized_;
    int width;int height;

    


    
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    base_local_planner::WorldModel* world_model_;
    double footprintCost(double x_i, double y_i, double theta_i);

    // Base functions for Planners
    void getCoordinate (float& x, float& y);
    void convertToCoordinate(int index, float& x, float& y);
    void mapToWorld(double mx, double my, double& wx, double& wy);
    void add_open(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[],int n);
   
    

    bool validate(float x, float y);
    bool isValid(int startCell,int goalCell); 
    bool isFree(int CellID); //returns true if the cell is Free

    int convertToCellIndex (float x, float y);
    int getIndex(int i,int j){return (i*width)+j;}  //occupancy grid map eka tiyenne list ekak wdht..meken list eke kothanada tiyenne kiyala denaw
    int getRow(int index){return index/width;}
    int getCol(int index){return index%width;}
    
    float getMoveCost(int CellID1, int CellID2);

    vector <int> getNeighbour (int CellID);
    vector<int> PathFinder(int startCell, int goalCell);
    vector<int> AStar(int startCell, int goalCell, float g_score[]);
    vector<int> Dijkstra(int startCell, int goalCell, float g_score[]);
    vector<int> BFS(int startCell, int goalCell, float g_score[]);
    vector<int> constructPath(int startCell, int goalCell, float g_score[]);


    //D-star-lite  related things
    vector<int> D_star_lite(int startcell_index, int goalcell_index, float g_and_rhs[][2]);

    vector <pair<node, pair<double, double>>> OPL;
    std::pair<node, std::pair<double, double>> OPL_top_key;

    int currentcell_index;
    int start_current_cell_index;
    pair <double, double> k_old;
    pair <double, double> k_new;

    std::pair<node, std::pair<double, double>> k_new_cell;

    void ComputeShortestPath(node start,node goal,float g_and_rhs[][2], int k_m);
    pair<node,pair<double, double>> CalculateKey(node current_cell,node start_current_cell,float g_and_rhs[][2], int k_m);
    pair<node, pair<double, double>> getTopKey();
    vector <int> constructPath_D_star_lite(int start_current_cell_index,int goal_cell_index, float g_and_rhs[][2]);

    
    float heuristic(int cellID, int goalCell, int n){
        int x1=getRow(goalCell);int y1=getCol(goalCell);int x2=getRow(cellID);int y2=getCol(cellID);
        int dx = abs(x2-x1) ; int dy = abs(y2-y1);
        // Manhattan Heuristic
        if(n==1)
            return dx + dy;
        // Euclidean Heuristic
        else if(n==2)
            return sqrt(dx*dx + dy*dy);

        // Chebyshev Heuristic
        else if(n==3)
            return max(dx,dy);

        // Octile Heuristic
        else
            return dx+dy+(sqrt(2)-2)*min (dx,dy);
    }

    float heuristic_from_s(int current_cell_index,int start_current_cell_index){  //This cell id means list eke row by row dammama keeweniyada kiyana eka
        int x1 = getRow(current_cell_index);
        int y1 = getCol(current_cell_index);
        int x2 = getRow(start_current_cell_index);
        int y2 = getCol(start_current_cell_index);

        int dx = abs(x1 - x2);
        int dy =abs(y1 - y2);
        // cout <<"dx "<< dx <<endl;
        // cout << "dy "<< dy <<endl;
        // cout << dx+dy <<endl;
        return dx+dy;//Considering Manhatten Distance
}





};
};
#endif

   

