void ComputeShortestPath(node start, node goal, float g_and_rhs[][2], int k_m)
{

	while ((g_and_rhs[start.cell_index][0] != g_and_rhs[start.cell_index][1]) || getTopKey().second < CalculateKey(start, start, g_and_rhs, k_m).second)
	{
		//since the getTopKey function is called in the while loop,  OPL_top_key is automatically calculated there
		k_old = OPL_top_key.second;

		node u = OPL_top_key.first; //current cell
		currentcell_index = u.cell_index;
		OPL.erase(OPL.begin());

		k_new_cell = CalculateKey(u, start, g_and_rhs, k_m);
		k_new = k_new_cell.second;

		if (k_old < k_new)
		{
			// OPL_top_key.second = k_new;
			OPL.push_back(k_new_cell);
		}

		else if (g_and_rhs[currentcell_index][0] > g_and_rhs[currentcell_index][1])
		{
			//  std::cout << "Hey" << std::endl;
			// cout << g_and_rhs[currentcell_index][1] <<endl;
			g_and_rhs[currentcell_index][0] = g_and_rhs[currentcell_index][1]; //setting g = rhs
			// OPL.erase(OPL.begin());
			// cout << g_and_rhs[currentcell_index][0] <<endl;
			std::vector<int> neighbour_predecessors;
			neighbour_predecessors = getNeighbour(currentcell_index);

			for (uint i = 0; i < neighbour_predecessors.size(); i++)
			{
				// std::cout << "Hey1" << std::endl;
				node predecessor;
				predecessor.cell_index = neighbour_predecessors[i];
				// cout << predecessor.cell_index << endl;

				std::vector<int> neighbour_succesors;
				if (neighbour_predecessors[i] != goal.cell_index)
				{
					// cout << "In here" << endl;
					neighbour_succesors = getNeighbour(neighbour_predecessors[i]);
					float min_rhs = infinity;
					for (uint j = 0; j < neighbour_succesors.size(); j++)
					{
						min_rhs = std::min(min_rhs, g_and_rhs[neighbour_succesors[j]][0] + getMoveCost(neighbour_predecessors[i], neighbour_succesors[j]));
					}
					g_and_rhs[neighbour_predecessors[i]][1] = min_rhs; //predecessor kenekt enna puluwn aduma cost path eka eyage successorlagen update karagnnwa
				}
				UpdateVertex(predecessor, start, g_and_rhs, k_m);
			}
		}

		else if (g_and_rhs[currentcell_index][0] < g_and_rhs[currentcell_index][1])
		{ //Underconsistant
			float g_old = g_and_rhs[currentcell_index][0];
			g_and_rhs[currentcell_index][0] = infinity;

			std::vector<int> neighbour_predecessors;
			neighbour_predecessors = getNeighbour(currentcell_index);

			//Updating the current cell 
			if (currentcell_index != goal.cell_index)
			{
				neighbour_succesors = getNeighbour(currentcell_index);
				float min_rhs = infinity;
				for (uint j = 0; j < neighbour_succesors.size(); j++)
				{
					min_rhs = std::min(min_rhs, g_and_rhs[neighbour_succesors[j]][0] + getMoveCost(currentcell_index, neighbour_succesors[j]));
				}
				g_and_rhs[currentcell_index][1] = min_rhs;
			}

			UpdateVertex(u, start, g_and_rhs, k_m);

			//Updating the predecessors
			for (uint i = 0; i < neighbour_predecessors.size(); i++)
			{
				node predecessor;
				predecessor.cell_index = neighbour_predecessors[i];
				std::vector<int> neighbour_succesors;

				if (neighbour_predecessors[i] != goal.cell_index)
				{
					neighbour_succesors = getNeighbour(neighbour_predecessors[i]);
					float min_rhs = infinity;
					for (uint j = 0; j < neighbour_succesors.size(); j++)
					{
						min_rhs = std::min(min_rhs, g_and_rhs[neighbour_succesors[j]][0] + getMoveCost(neighbour_predecessors[i], neighbour_succesors[j]));
					}
					g_and_rhs[neighbour_predecessors[i]][1] = min_rhs;
				}

				UpdateVertex(predecessor, start, g_and_rhs, k_m);
			}
		}
	}

	// g_and_rhs[start.cell_index][0] = g_and_rhs[start.cell_index][1];
}
