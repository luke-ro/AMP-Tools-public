#include "CSpace2D.h"

CSpace2D::CSpace2D(double x0_min, double x0_max, double x1_min, double x1_max)
 :ConfigurationSpace2D(x0_min,x0_max,x1_min,x1_max){};

/**
 * Takes in a 2D  obstacle and robot and returns the minkowski difference
 * 
 * @param obstacle the polygon describing the obstacle in the workspace
 * @param robot the polygon describing the robot in the workspace
 * @return the polygon of the C-Space obstacle
*/
amp::Polygon CSpace2D::minkDiff(const amp::Polygon& obstacle,  const amp::Polygon& rob_pos){
    amp::Polygon rob = pgDifference(rob_pos); 
    rob = zeroPgCCW(rob);
    amp::Polygon obs = zeroPgCCW(obstacle);

    int i=1;
    int n=obs.verticesCCW().size();

    int j=1;
    int m=rob.verticesCCW().size();

    std::vector<Eigen::Vector2d> verts;
    do{
        verts.push_back(obs.verticesCCW()[i]+rob.verticesCCW()[j]);
        if (ang02pi(obs.verticesCCW()[i%n],obs.verticesCCW()[(i+1)%n]) > ang02pi(rob.verticesCCW()[j%m],rob.verticesCCW()[(j+1)%m])){
            i++;
        }else if(ang02pi(obs.verticesCCW()[i%n],obs.verticesCCW()[(i+1)%n]) < ang02pi(rob.verticesCCW()[j%m],rob.verticesCCW()[(j+1)%m])){
            j++;
        }else{
            i++;
            j++;
        }
    }while(!((i==n+1) && (j==m+1)));

    return amp::Polygon(verts);
}


/**
 * Takes in a 2D polygon and returns the index of lowest (and leftmost if tied) vertex
 * 
 * @param pg the polygon 
 * @return the index of the lowest (then leftmost if tie) vertex
*/
int CSpace2D::botLeftVerCCW(const amp::Polygon& pg){
    int i=0;
    int min_i=-1;
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
 * Return the negative of a polygon
 * 
 * @param pg polygon to find the negative of
 * @return the negative of the polygon
*/
amp::Polygon CSpace2D::pgDifference(const amp::Polygon& pg){
    std::vector<Eigen::Vector2d> verts;
    for(int i=pg.verticesCCW().size()-1; i>=0; i--){
        verts.push_back(-pg.verticesCCW()[i]);
    }
    return amp::Polygon(verts); 
}

/**
 * puts the lowest left vertex on the origin (0,0)
 * 
 * @param pg a convex polygon
 * @return a new polygon with first vertex as bottom left vertex on origin. 
*/
amp::Polygon CSpace2D::zeroPgCCW(const amp::Polygon& pg) { 
    int offset = botLeftVerCCW(pg);
    Eigen::Vector2d botLeft = pg.verticesCCW()[offset];

    std::vector<Eigen::Vector2d> verts;
    int sz = pg.verticesCCW().size();

    for(int i=0; i<sz;i++){
        verts.push_back(pg.verticesCCW()[(i+offset)%sz]-botLeft);
    }

    return amp::Polygon(verts);
}
