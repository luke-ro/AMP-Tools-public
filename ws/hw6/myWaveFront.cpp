#include "myWaveFront.h"

inline int wrapIdxs(int i, int j, int rowlength){
    return (i*rowlength)+j;
}

inline std::pair<int,int> unwrapIdx(int k, int rowlength){
    return std::pair<int,int>(k/rowlength,k%rowlength);
}

inline std::vector<int> getAdjCells(int c, std::pair<int,int> dims){
    std::pair<int,int>  ij = unwrapIdx(c,dims.first);
    int i = ij.first;
    int j = ij.second;
    std::vector<int> cells;
    if(i>0)
        cells.push_back(wrapIdxs(i-1, j, dims.first));

    if(i<dims.first-1) 
        cells.push_back(wrapIdxs(i+1, j, dims.first));

    if(j>0)
        cells.push_back(wrapIdxs(i, j-1, dims.first));
    
    if(j<dims.second)
        cells.push_back(wrapIdxs(i, j+1, dims.first));

    return cells;
}

/**
 * @brief plans through a discrete world using wavefron
*/
amp::Path2D myWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    std::pair<int,int> dims = grid_cspace.size();
    
    //get a map(hash table) that stores if each cell has or has not been visited.  
    std::unordered_map<int,bool> visited;
    for(int k=0; k<(dims.first*dims.second);k++){
        visited[k] = false;
    }
    
    std::list<int> q; //fill with cell numbers (i*rowsize+j)
                        // will have to go back and forth from cell nmber to idx

    // int idx0_init = H::numToIdx(q_init[0],_x0_bounds[0],_x0_bounds[1],_sz_x0);
    // int idx1_init = H::numToIdx(q_init[1],_x1_bounds[0],_x1_bounds[1],_sz_x1);

    //Assume that we get an index somehow?
    int idx0_init = 0;
    int idx1_init = 0;
     
    q.push_back(wrapIdxs(idx0_init,idx1_init,dims.first));

    while(!q.empty()){
        int curr = q.front();
        // This is where the cell needs to have its value inserted.
        // Just add one to the minimum neighboring value? 
        q.pop_front();

        for(auto adj : getAdjCells(curr,dims)){
            if(!visited[adj]){
                visited[adj] = true;
                q.push_back(adj);
            }
        }
    }


    return amp::Path2D();
}

/**
 * @brief Constructs a discrete occupancy grid based on an environmnet
*/
std::unique_ptr<amp::GridCSpace2D> myWaveFront::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    _x0_bounds[0] = environment.x_min;
    _x0_bounds[1] = environment.x_max;
    _x1_bounds[0] = environment.y_min;
    _x1_bounds[1] = environment.y_max;
    
    _sz_x0 = floor(abs(environment.x_max-environment.x_min)/0.25);
    _sz_x1 = floor(abs(environment.y_max-environment.y_min)/0.25);
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
