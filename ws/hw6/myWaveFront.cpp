#include "myWaveFront.h"

inline int wrapIdxs(int i, int j, int rowlength){
    return (i*rowlength)+j;
}

inline std::pair<int,int> unwrapIdx(int k, int rowlength){
    return std::pair<int,int>(k/rowlength,k%rowlength);
}

inline std::vector<std::pair<int,int>> getAdjCells(int i, int j, std::pair<int,int> dims){
    std::vector<std::pair<int,int>> cells;
    if(i>0){
        cells.push_back(std::pair<int,int>(i-1,j));
    }

    if(i<dims.first-1){
        cells.push_back(std::pair<int,int>(i+1,j));
    }

    if(j>0){
        cells.push_back(std::pair<int,int>(i,j-1));
    }
    
    if(j<dims.second-1){
        cells.push_back(std::pair<int,int>(i,j+1));
    }

    return cells;
}

inline int minNeighbor(int i, int j, const amp::DenseArray2D<int>& arr){
    std::pair<int,int> dims = arr.size();
    std::vector<std::pair<int,int>> neighbors = getAdjCells(i, j, dims);
    int min_val = 10000000;
    for(auto nb : neighbors){
        int temp = arr(nb.first,nb.second);
        if(temp<min_val && temp>0){
            min_val = temp;
        }
    }
    return min_val;
}

inline std::pair<int,int> minNeighborIdx(int i, int j, const amp::DenseArray2D<int>& arr){
    std::pair<int,int> dims = arr.size();
    std::vector<std::pair<int,int>> neighbors = getAdjCells(i, j, dims);
    int min_val = 1000000;
    std::pair<int,int> idx_min  = neighbors[0];
    for(auto nb : neighbors){
        int temp = arr(nb.first,nb.second);
        if(temp<min_val && temp>0){
            min_val = temp;
            idx_min = nb;
        }
    }
    return idx_min;
}

/**
 * @brief plans through a discrete world using wavefron
*/
amp::Path2D myWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    std::pair<int,int> dims = grid_cspace.size();
    std::pair<int,int>  x0_bounds = grid_cspace.x0Bounds();
    std::pair<int,int>  x1_bounds = grid_cspace.x1Bounds();
    
    //get a map(hash table) that stores if each cell has or has not been visited.  
    std::unordered_map<int,bool> visited;

    // array that will store wavefront values
    amp::DenseArray2D<int> wave(dims.first, dims.second);

    //initialize visited and wave
    Eigen::Vector2d q_check;
    for(int k=0; k<(dims.first*dims.second);k++){
        std::pair<int,int> ij = unwrapIdx(k,dims.second);

        // if obstacle, visited=true and set equal to 1
        if(grid_cspace(ij.first, ij.second)){
            visited[k] = true;
            wave(ij.first, ij.second) = INT_MAX;

        // Else: free space, set to 0
        }else{
            visited[k] = false;
            wave(ij.first, ij.second) = 0;
        }

    }

    
    std::list<int> queue; //fill with cell numbers (i*rowsize+j)
                        // will have to go back and forth from cell nmber to idx

    // int idx0_init = H::numToIdx(q_init[0],_x0_bounds[0],_x0_bounds[1],_sz_x0);
    // int idx1_init = H::numToIdx(q_init[1],_x1_bounds[0],_x1_bounds[1],_sz_x1);

    //Assume that we get an index somehow?
    std::pair<int,int> idx_goal;
    idx_goal.first = H::numToIdx(q_goal[0],x0_bounds.first,x0_bounds.second,dims.first);
    idx_goal.second = H::numToIdx(q_goal[1],x1_bounds.first,x1_bounds.second,dims.second);
    wave(idx_goal.first, idx_goal.second)=2;

    std::pair<int,int> idx;

    //need to start loop not on the first cell
    std::vector<std::pair<int,int>> neighbors = getAdjCells(idx_goal.first,idx_goal.second,dims);
    visited[wrapIdxs(idx_goal.first, idx_goal.second, dims.second)] = true;
    queue.push_back(wrapIdxs(neighbors[0].first, neighbors[0].second, dims.second));

    while(!queue.empty()){
        int curr = queue.front();
        queue.pop_front();

        // This is where the cell needs to have its value inserted.
        // Just add one to the minimum neighboring value? 
        idx = unwrapIdx(curr,dims.second);
        wave(idx.first,idx.second) = 1+minNeighbor(idx.first, idx.second, wave);

        int k;
        for(auto adj : getAdjCells(idx.first, idx.second, dims)){
            k = wrapIdxs(adj.first, adj.second, dims.second);
            std::pair<int,int> test = unwrapIdx(k,dims.second);
            if(!visited[k]){
                visited[k] = true;
                queue.push_back(k);
            }
        }
    }
    std::cout << "here" << "\n";

    // plan through it
    amp::Path2D path;
    Eigen::Vector2d q;
    std::pair<int,int> idx_path;
    idx_path.first = H::numToIdx(q_init[0],x0_bounds.first,x0_bounds.second,dims.first);
    idx_path.second = H::numToIdx(q_init[1],x1_bounds.first,x1_bounds.second,dims.second);
    
    while(idx_path!=idx_goal){
        q[0] = H::idxToNum(idx_path.first,dims.first,x0_bounds.first,x0_bounds.second);
        q[1] = H::idxToNum(idx_path.second,dims.second,x1_bounds.first,x1_bounds.second);
        path.waypoints.push_back(q);

        idx_path = minNeighborIdx(idx_path.first,idx_path.second,wave);
    }


    return path;
}

/**
 * @brief Constructs a discrete occupancy grid based on an environmnet
*/
std::unique_ptr<amp::GridCSpace2D> myWaveFront::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    _x0_bounds[0] = environment.x_min;
    _x0_bounds[1] = environment.x_max;
    _x1_bounds[0] = environment.y_min;
    _x1_bounds[1] = environment.y_max;
    
    _sz_x0 = floor(abs(environment.x_max-environment.x_min)/_cell_width);
    _sz_x1 = floor(abs(environment.y_max-environment.y_min)/_cell_width);
    auto grid_ptr = std::make_unique<CSpace2D>(environment.x_min,
            environment.x_max,
            environment.y_min,
            environment.y_max,
            _sz_x0,_sz_x1);

    Eigen::Vector2d q_ij;
    for(int i=0; i<_sz_x0; i++){
        q_ij[0] = H::idxToNum(i, _sz_x0, environment.x_min, environment.x_max);
        for(int j=0; j<_sz_x1; j++){
            q_ij[1] = H::idxToNum(j, _sz_x1, environment.y_min, environment.y_max);   
            if(H::checkCollsionEnv(environment,q_ij)){
                (*grid_ptr)(i,j) = true;
            }else{
                (*grid_ptr)(i,j) = false;
            }
        }
    }
    return grid_ptr;
}
