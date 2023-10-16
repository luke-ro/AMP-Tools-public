#include "myWaveFront.h"

/**
 * @brief plans through a discrete world using wavefron
*/
amp::Path2D myWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    return amp::Path2D();
}

/**
 * @brief Constructs a discrete occupancy grid based on an environmnet
*/
std::unique_ptr<amp::GridCSpace2D> myWaveFront::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    int sz_x0 = floor(abs(environment.x_max-environment.x_min)/0.25);
    int sz_x1 = floor(abs(environment.y_max-environment.y_min)/0.25);
    auto grid_ptr = std::make_unique<CSpace2D>(environment.x_min,
            environment.x_max,
            environment.y_min,
            environment.y_max,
            sz_x0,sz_x1);

    Eigen::Vector2d q_ij;
    for(int i=0; i<sz_x0; i++){
        q_ij[0] = H::idxToNum(i, sz_x0, environment.x_min, environment.x_max);
        for(int j=0; j<sz_x0; j++){
            q_ij[1] = H::idxToNum(j, sz_x1, environment.y_min, environment.y_max);   
            if(H::checkCollsionEnv(environment,q_ij)){
                (*grid_ptr)(i,j) = true;
            }
        }
    }
    return grid_ptr;
}
