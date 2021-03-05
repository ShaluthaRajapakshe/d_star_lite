#include "Dstar_lite.h"

PLUGINLIB_EXPORT_CLASS(PathPlanners_all::PathPlannersROS, nav_core::BaseGlobalPlanner);

int mapSize;
bool* OGM;


static const float INFINIT_COST = INT_MAX;
float infinity = std::numeric_limits< float >::infinity();

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end){
	
	timespec temp;
	
	if ((end.tv_nsec-start.tv_nsec)<0){
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
		//aded code
	}
	
	else{
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}


inline vector <int> getNeighbour (int CellID);


namespace PathPlanners_all
{
PathPlannersROS::PathPlannersROS(){}
PathPlannersROS::PathPlannersROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros){initialize(name, costmap_ros);}

void PathPlannersROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	if (!initialized_){
		costmap_ros_ = costmap_ros;
		costmap_ = costmap_ros_->getCostmap();

		//added using carrotplanner
		ros::NodeHandle private_nh("~/" + name);
		private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
		// world_model_ = new base_local_planner::CostmapModel(*costmap_); 

		originX = costmap_->getOriginX();
		originY = costmap_->getOriginY();

		width = costmap_->getSizeInCellsX();
		height = costmap_->getSizeInCellsY();
		resolution = costmap_->getResolution();
		mapSize = width*height;
		

		unsigned int max_cost = 0;
		OGM = new bool [mapSize]; 
		for (unsigned int iy = 0; iy < height; iy++){
			for (unsigned int ix = 0; ix < width; ix++){
				unsigned int cost = static_cast<int>(costmap_->getCost(ix,iy));
				
				if (cost > max_cost){
					max_cost = cost;
				}

				if (cost <= 252){ //220
					OGM[iy*width+ix]=true;
					// cout <<"Traversable"<< ix<<","<<iy<<"   cost:"<<cost;
					// ROS_INFO("cost %d  %d  cost: %d", ix, iy, cost);
				}

				else{
					OGM[iy*width+ix]=false;
					ROS_INFO("Pbstacle %d  %d  cost: %d", ix, iy, cost);
					// cout <<"Obstacle"<< ix<<","<<iy<<"   cost:"<<cost;
				}
				
			}
			cout << max_cost << endl;;
		}
		ROS_INFO("Jump Point Search initialized successfully");
		initialized_ = true;
	}
	else
    	ROS_WARN("Planner already initialized");
}

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double PathPlannersROS::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


bool PathPlannersROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
	if (!initialized_){
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
		return false;
	}


	
	ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
	plan.clear();

	
	// costmap_ = costmap_ros_->getCostmap();
	// originX = costmap_->getOriginX();
	// originY = costmap_->getOriginY();

	// width = costmap_->getSizeInCellsX();
	// height = costmap_->getSizeInCellsY();
	// resolution = costmap_->getResolution();
	// mapSize = width*height;

	// OGM = new bool [mapSize]; 
	// for (unsigned int iy = 0; iy < height; iy++){
	// 	for (unsigned int ix = 0; ix < width; ix++){
	// 		unsigned int cost = static_cast<int>(costmap_->getCost(ix,iy));
				
	// 		if (cost <= 220){ //220
	// 			OGM[iy*width+ix]=true;
	// 				// cout <<"Traversable"<< ix<<","<<iy<<"   cost:"<<cost;
	// 				// ROS_INFO("cost %d  %d  cost: %d", ix, iy, cost);
	// 		}

	// 		else{
	// 			OGM[iy*width+ix]=false;
	// 			ROS_INFO("Pbstacle %d  %d  cost: %d", ix, iy, cost);
	// 				// cout <<"Obstacle"<< ix<<","<<iy<<"   cost:"<<cost;
	// 		}
				
	// 	}
	// 	cout << endl;
	// }



	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){  //goal frame and global frame shoudl be the same
		ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
		costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}

	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;

	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	getCoordinate(startX, startY);
	getCoordinate(goalX, goalY);

	int startCell;
	int goalCell;

	if (validate(startX, startY) && validate(goalX, goalY)){
		startCell = convertToCellIndex(startX, startY);
		goalCell = convertToCellIndex(goalX, goalY);
	}

	else{
		ROS_WARN("the start or goal is out of the map");
		return false;
	}

	if (isValid(startCell, goalCell)){
		vector<int> bestPath;
		bestPath.clear();
		bestPath = PathFinder(startCell, goalCell);
		if(bestPath.size()>0){
			for (int i = 0; i < bestPath.size(); i++){
				float x = 0.0;
				float y = 0.0;
				float previous_x = 0.0;
        		float previous_y = 0.0;

				int previous_index;

				int index = bestPath[i];
				convertToCoordinate(index, x, y);

				if (i != 0)
        		{
          			previous_index = bestPath[i - 1];
        		}
        		else
        		{
          			previous_index = index;
        		}
				convertToCoordinate(previous_index, previous_x, previous_y);

				 //Orient the robot towards target
        		tf::Vector3 vectorToTarget;
        		vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
        		float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

				geometry_msgs::PoseStamped pose = goal;

				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				// pose.pose.orientation.x = 0.0;
				// pose.pose.orientation.y = 0.0;
				// pose.pose.orientation.z = 0.0;
				// pose.pose.orientation.w = 1.0;

				// pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

				plan.push_back(pose);
			}

			float path_length = 0.0;
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;

			for (; it!=plan.end();++it){
				path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y );
				last_pose = *it;
			}

			cout <<"The global path length: "<< path_length<< " meters"<<endl;
			return true;
		}
		else{
			ROS_WARN("The planner failed to find a path, choose other goal position");
			return false;
		}
	}
	
	else{
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

void PathPlannersROS::getCoordinate(float& x, float& y){
	x = x - originX;
	y = y - originY;
}

int PathPlannersROS::convertToCellIndex(float x, float y){
	int cellIndex;
	float newX = x / resolution;
	float newY = y / resolution;
	cellIndex = getIndex(newY, newX);
	return cellIndex;
}

void PathPlannersROS::convertToCoordinate(int index, float& x, float& y){
	x = getCol(index) * resolution;
	y = getRow(index) * resolution;
	x = x + originX;
	y = y + originY;
}

bool PathPlannersROS::validate(float x, float y){
	bool valid = true;
	if (x > (width * resolution) || y > (height * resolution))
		valid = false;
	return valid;
}

void PathPlannersROS::mapToWorld(double mx, double my, double& wx, double& wy){
	costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
	wx = costmap->getOriginX() + mx * resolution;
	wy = costmap->getOriginY() + my * resolution;
}

vector<int> PathPlannersROS::PathFinder(int startCell, int goalCell){
	vector<int> bestPath;
	float g_score [mapSize];
	for (uint i=0; i<mapSize; i++)
		g_score[i]=infinity;

	//D_star_lite
	float g_and_rhs [mapSize][2];
 //Initializing g and rhs values
  	for (int i = 0; i< mapSize;i++){
  	g_and_rhs[i][0] = infinity; //g
  	g_and_rhs[i][1] = infinity; //rhs
 	}

	timespec time1, time2;

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	
        bestPath=AStar(startCell, goalCell,  g_score);
	//bestPath = D_star_lite(startCell,goalCell, g_and_rhs);
	// bestPath=Dijkstra(startCell, goalCell,  g_score);
	// bestPath=BFS(startCell, goalCell,  g_score);
	// bestPath=JPS(startCell,goalCell,g_score);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

	cout<<" Time taken to generate path= " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;

	return bestPath;
}

vector<int> PathPlannersROS::AStar(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;   //list of integers
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;  //cells with node id and f cost 
	int currentCell;

	g_score[startCell]=0;   //starts by putting zero to the start cell's g value
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell]+heuristic(startCell,goalCell,1);  //d star lie eke key pair eken thamai ganne..mage eke man hadala tiyenwa cell id ekath ekka key_pair eka ganna puluwn wdht
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 1); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::Dijkstra(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;  //cells kiyana struct eke cell id eka gannaaw
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::BFS(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+1;
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}


vector<int> PathPlannersROS:: D_star_lite(int startCell_index, int goalCell_index, float g_and_rhs[][2]){
  vector <int> bestPath;
  vector <int> emptyPath;

  int k_m = 0;


  node goal;
  node start;

  goal.cell_index = goalCell_index;
  start.cell_index = startCell_index;

  g_and_rhs[goalCell_index][1] = 0; //setting rhs value of goal to zero


  start_current_cell_index = startCell_index;  //start_current_cell_index will be used when the robot moves..not needed to calculate the paths

//   // calculated_key = CalculateKey(goal, start);

  OPL.push_back(CalculateKey(goal, start, g_and_rhs, k_m));

  currentcell_index = goalCell_index;  //this is needed to calculate the path

  ComputeShortestPath(start, goal, g_and_rhs, k_m);


//   float final_grid [mapSize];
//   for (int i = 0; i <mapSize; i++){
//     final_grid[i] = g_and_rhs[i][0];  //getting the g values
//   }
  //we can debug from here by printing the final grid values

  //we have to start from goal and go to the minimum cost cell from there..like wise we can calculate the path

  if(g_and_rhs[startCell_index][0]!=infinity){
	bestPath=constructPath_D_star_lite(startCell_index, goalCell_index, g_and_rhs );  //have to check this function
	return bestPath; 
   }
	
  else{
	cout << "Path not found!" << endl;
	return emptyPath;
   }

//   cout << "Path Calculated" << endl;

//   return bestPath;

}

void PathPlannersROS::ComputeShortestPath(node start,node goal,float g_and_rhs[][2], int k_m){
	while (!OPL.empty() && ((g_and_rhs[start.cell_index][0] != g_and_rhs[start.cell_index][1]) || getTopKey().second < CalculateKey(start, start, g_and_rhs, k_m).second)){
		k_old = OPL_top_key.second; 

    	node u = OPL_top_key.first; //current cell
    	currentcell_index = u.cell_index;
		OPL.erase(OPL.begin());
		// getTopKey();
		cout << OPL_top_key.first.cell_index << endl;
		
		k_new_cell = CalculateKey(u, start,g_and_rhs, k_m);
		k_new = k_new_cell.second;

		if (k_old < k_new){
      		OPL.push_back(k_new_cell);
    	}


		if (g_and_rhs[currentcell_index][0] > g_and_rhs[currentcell_index][1] ){  //Overconsistent
      		//  std::cout << "Hey" << std::endl;
      		// cout << g_and_rhs[currentcell_index][1] <<endl;
      		g_and_rhs[currentcell_index][0] = g_and_rhs[currentcell_index][1]; //setting g = rhs
      		// cout << g_and_rhs[currentcell_index][0] <<endl;
      		std::vector <int> neighbour_predecessors;
      		neighbour_predecessors = getNeighbour(currentcell_index);

      		for (uint i = 0;i < neighbour_predecessors.size(); i++){
        		// std::cout << "Hey1" << std::endl;
        		node predecessor;
        		predecessor.cell_index = neighbour_predecessors[i];
        		// cout << predecessor.cell_index << endl;

        		std::vector <int> neighbour_succesors;
        		if (neighbour_predecessors[i] != goal.cell_index){
            	// cout << "In here" << endl;
            		neighbour_succesors = getNeighbour(neighbour_predecessors[i]);
            		float min_rhs = infinity;
            		for (uint j = 0;j < neighbour_succesors.size(); j++){
					// cout << neighbour_succesors[j] <<endl;
					// cout << g_and_rhs[neighbour_succesors[i]][0] <<endl;
					min_rhs = std::min(min_rhs, g_and_rhs[neighbour_succesors[j]][0] + getMoveCost(neighbour_predecessors[i],neighbour_succesors[j]));
              

            		}
            		g_and_rhs[neighbour_predecessors[i]][1] = min_rhs;//predecessor kenekt enna puluwn aduma cost path eka eyage successorlagen update karagnnwa
            // cout << "g: "<< g_and_rhs[predecessor.cell_index][0] << " rhs: "<< g_and_rhs[predecessor.cell_index][1] << endl;

        		}

        		if (g_and_rhs[predecessor.cell_index][0] != g_and_rhs[predecessor.cell_index][1]){
            
            		OPL.push_back(CalculateKey(predecessor, start, g_and_rhs,k_m));
        		}

       		}	


    	}


		if (g_and_rhs[currentcell_index][0] < g_and_rhs[currentcell_index][1]) { //Underconsistant
      		g_and_rhs[currentcell_index][0] = infinity;

      		std::vector <int> neighbour_predecessors;
      		neighbour_predecessors = getNeighbour(currentcell_index);

      		for (uint i = 0;i < neighbour_predecessors.size(); i++){
				node predecessor;
				predecessor.cell_index = neighbour_predecessors[i];
				std::vector <int> neighbour_succesors;
				if (neighbour_predecessors[i] != goal.cell_index){
					neighbour_succesors = getNeighbour(neighbour_predecessors[i]);
					float min_rhs = infinity;
					for (uint j = 0;j < neighbour_succesors.size(); j++){
						min_rhs = std::min(min_rhs, g_and_rhs[neighbour_succesors[j]][0] + getMoveCost(neighbour_predecessors[i],neighbour_succesors[j]));

					}
					g_and_rhs[neighbour_predecessors[i]][1] = min_rhs;

				}

				if (g_and_rhs[predecessor.cell_index][0] != g_and_rhs[predecessor.cell_index][1]){
					OPL.push_back(CalculateKey(predecessor, start, g_and_rhs,k_m));
				}

			}

    	}

	}
	g_and_rhs[start.cell_index][0] = g_and_rhs[start.cell_index][1];

}

pair<node, pair<double, double>> PathPlannersROS::CalculateKey(node current_cell,node start_current_cell, float g_and_rhs[][2], int k_m){

    std::pair <double, double> key_pair;

    int current_cell_index = current_cell.cell_index;
    int start_current_cell_index = start_current_cell.cell_index;
  

    key_pair = make_pair(min(g_and_rhs[current_cell_index][0],g_and_rhs[current_cell_index][1]) + heuristic_from_s(current_cell_index,start_current_cell_index) + k_m, min(g_and_rhs[current_cell_index][0],g_and_rhs[current_cell_index][1]));
  //  key = std::make_pair(1,1);
    pair<node, pair<double, double>> OPL_pair = make_pair(current_cell, key_pair);

// //  std::cout << key_pair.first.cell_index << std::endl;
    // std::cout << "key " << OPL_pair.second.first << " " << OPL_pair.second.second << std::endl;

return OPL_pair;  //this will return the values for priority queue..(cell, (key pair))
}

pair<node, pair<double, double>> PathPlannersROS::getTopKey(){
	
	
	for (int i = 1; i < OPL.size();i++){
    OPL_top_key = OPL[i];
    int j = i-1;
    while (j >= 0 && (OPL[j].second.first > OPL_top_key.second.first || (OPL[j].second.first == OPL_top_key.second.first && OPL[j].second.second >= OPL_top_key.second.second))){
      OPL[j+1] = OPL[j];
      j--;
    }
    OPL[j+1] = OPL_top_key;
  }

  OPL_top_key = OPL[0]; //without pop
  // top_key = OPL.pop_back

  return OPL_top_key;
}

vector <int> PathPlannersROS::constructPath_D_star_lite(int start_current_cell_index,int goal_cell_index, float g_and_rhs[][2]){
  vector <int> bestPath;
  vector <int> path;
  int next_cell_index;
  int previous_cell_index = goal_cell_index;
  
  path.insert(path.begin()+ bestPath.size(), goal_cell_index);
  int currentCell = goal_cell_index;

  while (currentCell != start_current_cell_index){
    
    float min_g = infinity;
    
    vector <int> neighbour_cells;
    neighbour_cells = getNeighbour(currentCell);

    for (uint i = 0;i < neighbour_cells.size();i++){
     
      if (neighbour_cells[i] == previous_cell_index){
        continue;
      }
      else{
      
        float cost_to_current_node = g_and_rhs[neighbour_cells[i]][0] + getMoveCost(neighbour_cells[i], currentCell);

        if (cost_to_current_node < min_g){
          min_g = cost_to_current_node;
          next_cell_index = neighbour_cells[i];
        
        }
      }

      
    }
    previous_cell_index = currentCell;
    // cout << next_cell_index << " " << min_g << endl;
    currentCell = next_cell_index;
    // cout << currentCell<< endl;
    path.insert(path.begin()+path.size(), currentCell);
    
  
  }


  for (uint i=0; i < path.size();i++){
    // std::cout << path[i] << ' ';
    
    bestPath.insert(bestPath.begin()+ bestPath.size(), path[path.size() - (i+1)]);
  }


  return bestPath;
}

vector<int> PathPlannersROS::constructPath(int startCell, int goalCell,float g_score[])
{
	vector<int> bestPath;
	vector<int> path;

	path.insert(path.begin()+bestPath.size(), goalCell);  //patg vector eke udama tiyenne goal eka
	int currentCell=goalCell;

	while(currentCell!=startCell){   //start cell eka enakn passen passata enawa
		vector <int> neighborCells;
		neighborCells=getNeighbour(currentCell);    //dan inna cell eke neighboursla hoyanawa etanin aduma g value eka tiyena kena select karaganna

		vector <float> gScoresNeighbors;  //adala neighbourge g values push karagannaw
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]]);

			// I think we have to implement next node in shortest path function here
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end())); //min element will give the minimmum eleement in that range, 
		currentCell=neighborCells[posMinGScore];

		path.insert(path.begin()+path.size(), currentCell);  //need to check how this works..implement this in repl
	}

	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

void PathPlannersROS::add_open(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[] ,int n){
	cells CP;
	CP.currentCell=neighborCell;
	if (n==1)
		CP.fCost=g_score[neighborCell]+heuristic(neighborCell,goalCell,1);
	else
		CP.fCost=g_score[neighborCell];
	OPL.insert(CP);
}

vector <int> PathPlannersROS::getNeighbour (int CellID){
	int rowID=getRow(CellID);
	int colID=getCol(CellID);
	int neighborIndex;
	vector <int>  freeNeighborCells;

	for (int i=-1;i<=1;i++)
		for (int j=-1; j<=1;j++){
			if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighborIndex = getIndex(rowID+i,colID+j);
				if(isFree(neighborIndex) )
					freeNeighborCells.push_back(neighborIndex);
			}
		}

	return  freeNeighborCells;
}

bool PathPlannersROS::isValid(int startCell,int goalCell){ 
	bool isvalid=true;
	
	bool isFreeStartCell=isFree(startCell);
	bool isFreeGoalCell=isFree(goalCell);
	
	if (startCell==goalCell){
		cout << "The Start and the Goal cells are the same..." << endl; 
		isvalid = false;
	}
	
	else{
		
		if(!isFreeStartCell && !isFreeGoalCell){
			cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		}
		
		else{
			if(!isFreeStartCell){
				cout << "The start is an obstacle..." << endl;
				isvalid = false;
			}
			else{
				if(!isFreeGoalCell){
					cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				}
				else{
					if (getNeighbour(goalCell).size()==0){
						cout << "The goal cell is encountred by obstacles... "<< endl;
						isvalid = false;
					}
					else{
						if(getNeighbour(startCell).size()==0){
							cout << "The start cell is encountred by obstacles... "<< endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}

return isvalid;
}

float  PathPlannersROS::getMoveCost(int CellID1, int CellID2){
	int i1=0,i2=0,j1=0,j2=0;

	i1=getRow(CellID1);
	j1=getCol(CellID1);
	i2=getRow(CellID2);
	j2=getCol(CellID2);

	float moveCost=INFINIT_COST;
	if(i1!=i2 && j1!=j2)
		moveCost=1.4;
	else
		moveCost=1;
	return moveCost;
} 

bool  PathPlannersROS::isFree(int CellID){
	return OGM[CellID];
} 
};

bool operator<(cells const &c1, cells const &c2){return c1.fCost < c2.fCost;}

   
