#include "myGDAlgo.h"

myGDAlgo::myGDAlgo(int sx0, int sx1, double epsilon, double dstar_goal, double zeta, double Qstar, double eta, double alpha)
 :amp::GDAlgorithm()
 ,_sx0 (sx0)
 ,_sx1 (sx1)
 ,_epsilon (epsilon)
 ,_dstar_goal (dstar_goal)
 ,_zeta (zeta)
 ,_Qstar (Qstar)
 ,_eta (eta)
 ,_alpha (alpha)
 {}

amp::Path2D myGDAlgo::plan(const amp::Problem2D& problem){
    // std::vector< std::vector<Eigen::Vector2d> > grad (_sx0,
    //     std::vector<Eigen::Vector2d>(_sx1));
    Eigen::Vector2d q = problem.q_init;
    amp::Path2D path;

    path.waypoints.push_back(q);
    while((q-problem.q_goal).norm()>_epsilon){
        q += -_alpha*calcGrad(problem,q);
        path.waypoints.push_back(q);
    }

    return path;
}

void myGDAlgo::fillGradient(const amp::Problem2D& problem, std::vector<std::vector<Eigen::Vector2d>> &grad_arr){
    Eigen::Vector2d q;
    // fill potential func with base potential
    for(int i=0; i<_sx0; i++){
        q[0] = H::idxToNum(i,_sx0,problem.x_min,problem.x_max);
        for(int j=0; j<_sx1; j++){
            q[1] = H::idxToNum(j,_sx1,problem.y_min,problem.y_max);
            grad_arr[i][j] = calcGrad(problem,q);
        }
    }
}

Eigen::Vector2d myGDAlgo::calcGrad(const amp::Problem2D& problem, const Eigen::Vector2d& q){
    Eigen::Vector2d vec(0.0,0.0);


    // set to base gradient func
    vec += gradUatt(problem, q);

    // add repuslive func
    vec += gradUrep(problem, q);

    return vec;
}

Eigen::Vector2d myGDAlgo::gradUatt(const amp::Problem2D& problem, Eigen::Vector2d q){
    double dist_goal = (q-problem.q_goal).norm();
    if(dist_goal<_dstar_goal){
        return _zeta*(q-problem.q_goal);
    }else{
        return _dstar_goal*_zeta*(q-problem.q_goal)/dist_goal;
    }
}

Eigen::Vector2d myGDAlgo::gradUrep(const amp::Problem2D& problem, Eigen::Vector2d q){
    Eigen::Vector2d c = H::obstaclesClosePt(problem,q);
    double dist = (q-c).norm();
    if(dist < _Qstar){
        return _eta*(1/_Qstar-1/dist)*(q-c)/(dist*dist);
    }

    //else
    return Eigen::Vector2d(0.0,0.0);
}