#include "CSpace2D.h"

const double pi = 3.1415;

CSpace2D::CSpace2D(double x0_min, double x0_max, double x1_min, double x1_max, int s0, int s1)
    :GridCSpace2D(s0, s1, x0_min, x0_max, x1_min, x1_max)
    ,c_arr(s0, s1, false)
    ,x0_len(s0)
    ,x1_len(s1){
    
    // need to make a function to fill c_arr
    // std::unique_ptr<amp::GridCSpace2D> genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);

};

/**
 * @brief creates a cspace from manipulator and env
 * 
 * @param manipulator
 * @param env
 * @return cspace in 2d occupancy array form 
*/
CSpace2D CSpace2D::genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
    // need to fill c_arr
    CSpace2D temp_cspace(env.x_min, env.x_max, env.y_max, env.y_max, x0_len, x1_len);
    std::pair<int,int> bounds = c_arr.size();

    for(int i=0; i<bounds.first; i++){
        for(int j=0; j<bounds.second; j++){
            if(checkCollision(manipulator, env, idxToNumx0(i), idxToNumx1(j))){
                c_arr(i,j) = true;
                temp_cspace(i,j) = true;
            }
        }
    }
    
    return temp_cspace;
}

/**
 * @brief checks for a collision in env with given manip angles
 * 
 * @param manipulator the manipulator
 * @param env the environment with convex pgs
 * @param theta1 first angle (relative to global x axis)
 * @param theta2 second angle (relative to global x axis)
*/
bool CSpace2D::checkCollision(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, double theta1, double theta2){
    int n = 200;
    std::vector<Eigen::Vector2d> joints;
    // for(double theta1=0; theta1<2.0*pi; theta1+=2.0*pi/n){
    //     for(double theta2=0; theta1<2.0*pi; theta1+=2.0*pi/n){
    if (checkColConfig(manipulator, env, theta1, theta2)){
        return true;
    }
    //     }
    // }
    return false;
}

bool CSpace2D::checkColConfig(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, double theta1, double theta2){
    std::vector<double> state;
    state.push_back(theta1);
    state.push_back(theta2);

    std::vector<Eigen::Vector2d> joints;
    // joints[0] = manipulator.getJointLocation(state, 0);
    // joints[1] = manipulator.getJointLocation(state, 1);
    // joints[2] = manipulator.getJointLocation(state, 2);
    joints.push_back(manipulator.getJointLocation(state, 0));
    joints.push_back(manipulator.getJointLocation(state, 1));
    joints.push_back(manipulator.getJointLocation(state, 2));

    for(int k=0; k<2; k++){
        std::vector<Eigen::Vector2d> pts = H::linspace2D(joints[k],joints[k+1],20);
        for (auto pt : pts){
            if (H::checkCollsionEnv(env,pt)){
                return true;
            }
        }
    }
    return false;
}


/**
 * @brief Takes in a 2D  obstacle and robot and returns the minkowski difference
 * 
 * @param obstacle the polygon describing the obstacle in the workspace
 * @param robot the polygon describing the robot in the workspace
 * @return the polygon of the C-Space obstacle
*/
amp::Polygon CSpace2D::minkDiff(const amp::Polygon& obstacle,  const amp::Polygon& rob_pos){
    amp::Polygon rob = pgDifference(rob_pos); 
    rob = reorderPGCCW(rob, false);
    amp::Polygon obs = reorderPGCCW(obstacle);

    int i=0;
    int n=obs.verticesCCW().size();

    int j=0;
    int m=rob.verticesCCW().size();

    std::vector<Eigen::Vector2d> verts;
    do{
        verts.push_back(obs.verticesCCW()[i%n]+rob.verticesCCW()[j%m]);
        if ((ang02pi(obs.verticesCCW()[i%n],obs.verticesCCW()[(i+1)%n])+ (i>=n ? 2*3.1415:0 )) < (ang02pi(rob.verticesCCW()[j%m],rob.verticesCCW()[(j+1)%m]) + (j>=m ? 2*3.1415:0 ))){
            i++;
        }else if((ang02pi(obs.verticesCCW()[i%n],obs.verticesCCW()[(i+1)%n])+ (i>=n ? 2*3.1415:0 )) > (ang02pi(rob.verticesCCW()[j%m],rob.verticesCCW()[(j+1)%m]) + (j>=m ? 2*3.1415:0 ))){
            j++;
        }else{
            i++;
            j++;
        }
    }while(!((i==n) && (j==m)));

    return amp::Polygon(verts);
}

/**
 * @brief Returns if a point is in collision or not
 * 
 * @param x0
 * @param x1
 * @return true if in collision, false otherwise
*/
bool CSpace2D::inCollision(double x0, double x1) const{
    int i = (x0/(m_x0_bounds.second-m_x0_bounds.first))*(c_arr.size().first);
    int j = (x1/(m_x1_bounds.second-m_x1_bounds.first))*(c_arr.size().second);
    return c_arr(i,j);
}

/**
 * @brief Takes in a 2D polygon and returns the index of lowest (and leftmost if tied) vertex
 * 
 * @param pg the polygon 
 * @return the index of the lowest (then leftmost if tie) vertex
*/
int CSpace2D::botLeftVerCCW(const amp::Polygon& pg){
    int i=0;
    int min_i=0;
    Eigen::Vector2d min_ver=pg.verticesCCW()[0];

    for(auto ver : pg.verticesCCW()){
        // Check if y is lowest coord
        if (ver[1]<min_ver[1]){
            min_ver = ver;
            min_i = i;

        // if the y's are equal, check x's
        }else if (ver[1]==min_ver[1] && ver[0]<min_ver[0]){
            min_ver = ver;
            min_i = i;
        }
        i++;
    }
    return min_i;
}

/**
 * @brief Return the negative of a polygon
 * 
 * @param pg polygon to find the negative of
 * @return the negative of the polygon
*/
amp::Polygon CSpace2D::pgDifference(const amp::Polygon& pg){
    std::vector<Eigen::Vector2d> verts;
    for(int i=pg.verticesCCW().size()-1; i>=0; i--){
        verts.push_back(-pg.verticesCW()[i]);
    }
    return amp::Polygon(verts); 
}

/**
 * @brief puts the lowest left vertex on the origin (0,0)
 * 
 * @param pg a convex polygon
 * @return a new polygon with first vertex as bottom left vertex on origin. 
*/
amp::Polygon CSpace2D::reorderPGCCW(const amp::Polygon& pg, bool zero) { 
    int offset = botLeftVerCCW(pg); // Fix this func (?)
    Eigen::Vector2d botLeft = pg.verticesCCW()[offset];

    std::vector<Eigen::Vector2d> verts;
    int sz = pg.verticesCCW().size();

    if (zero){
        for(int i=0; i<sz;i++){
            verts.push_back(pg.verticesCCW()[(i+offset)%sz]-botLeft);
        }
    }else{
        for(int i=0; i<sz;i++){
            verts.push_back(pg.verticesCCW()[(i+offset)%sz]);
        }
    }
    

    return amp::Polygon(verts);
}
