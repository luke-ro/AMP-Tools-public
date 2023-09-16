#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helpers.h"
#include "cmath"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        MyBugAlgorithm() : _epsilon(0.0001) {};
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        bool checkCollision(const amp::Problem2D& problem, Eigen::Vector2d x1, Eigen::Vector2d x2);
    
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
    private:
        // Add any member variables here...
        const double _epsilon; 
};