#include "CSpace2D.h"

/**
 * Takes in a 2D  obstacle and robot and returns the minkowski difference
 * 
 * @param obstacle the polygon describing the obstacle in the workspace
 * @param robot the polygon describing the robot in the workspace
 * @return the polygon of the C-Space obstacle
*/
amp::Polygon minkDiff(const amp::Polygon& obs,  const amp::Polygon& rob_pos){
    amp::Polygon rob = pgDifference(rob_pos); 
    rob = zeroPgCCW(robot_neg);

    int i=1;
    int n=obs.verticesCCW.size();

    int j=1;
    int n=rob.verticesCCW.size();

    std::vector<Eigen::Vector2d> verts;
    do{
        verts.push_back(obs.verticesCCW[i]+rob.verticesCCW[j]);
        if 
    }while(!((i==n+1) && (j==m+1)))
}

/**
 * Return the negative of a polygon
 * 
 * @param pg polygon to find the negative of
 * @return the negative of the polygon
*/
amp::Polygon pgDifference(const amp::Polygon& pg){
    std::vector<Eigen::Vector2d> verts;
    for(int i=pg.verticesCCW.size()-1; i>=0; i--){
        verts.pushback(-g.verticesCCW[i]);
    }
    return amp::Polygon(verts); 
}

/**
 * Takes in a 2D polygon and returns the index of lowest (and leftmost if tied) vertex
 * 
 * @param pg the polygon 
 * @return the index of the lowest (then leftmost if tie) vertex
*/
int botLeftVerCCW(const amp::Polygon& pg){
    int i=0;
    min_i=-1;
    Eigen::Vector2d min_ver=pg.verticesCCW[0];

    for(auto const& ver : pg.verticesCCW){
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
 * puts the lowest left vertex on the origin (0,0)
 * 
 * @param pg a convex polygon
 * @return a new polygon with first vertex as bottom left vertex on origin. 
*/
amp::Polygon zeroPgCCW(amp::Polygon pg){
    int offset = botLeftVerCCW(pg);
    Eigen::Vector2d botLeft = pg.verticesCCW[offset];

    std::vector<Eigen::Vector2d> verts;
    int sz = pg.verticesCCW.size();

    for(int i=0; i<sz;i++){
        verts.pushback(pg[(i+offset)%sz]-botLeft)
    }

    return amp::Polygon(verts);
}

void MyClass::hereIsAMethod() {
    // Implementation
}