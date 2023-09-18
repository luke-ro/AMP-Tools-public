#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helpers.h"
#include "cmath"
#include "math.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        MyBugAlgorithm() : _epsilon(0.1), D_theta(0.01) {};
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // checks for collisions in entire ws
        bool isCollsion(const amp::Problem2D& problem, Eigen::Vector2d x);
        
        // Checks for a collision inside of a single polygon
        bool insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q);

        //determines if at goal within _epsilon distance. 
        bool atGoal(const amp::Problem2D& problem, const Eigen::Vector2d& x){return (problem.q_goal-x).norm() < _epsilon;};

        //determines if points are within _epsilon of eachother
        bool atPoint(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2){return (x1-x2).norm() < _epsilon;};

        // returns euclidian distance to goal
        double distToGoal(const amp::Problem2D& problem, const Eigen::Vector2d& x){return (problem.q_goal-x).norm();};

        // returns a small step directly towards the goal
        Eigen::Vector2d stepToGoal(const amp::Problem2D& problem, const Eigen::Vector2d& x){return (problem.q_goal-x).normalized()*_epsilon;}

        // returns a vector with _epsilon magnitude step around boundary
        Eigen::Vector2d borderFollowLeft(const amp::Problem2D& problem, const Eigen::Vector2d& q, const Eigen::Vector2d& q_prev);

        // gets orientation of vetor with 0 deg at x axis 
        double getAngle(Eigen::Vector2d vec){return atan2(vec[1],vec[0]);}

        //rotate vector by theta
        Eigen::Vector2d rotateVec(Eigen::Vector2d vec, double ang);


    private:
        // Add any member variables here...
        const double _epsilon; 
        const double D_theta;
};