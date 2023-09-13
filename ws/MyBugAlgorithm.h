#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helpers.h"
#include "cmath"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    MyBugAlgorithm(){_epsilon=0.0001;};

    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) const override;

        // Add any other methods here...
        bool checkCollision(const amp::Problem2D& problem, Eigen::vector2d x1, Eigen::vector2d x2);
    
        // checks for collisions in entire ws
        bool isCollsion(const amp::Problem2D& problem, Eigen::vector2d x);
        
        // Checks for a collision inside of a single polygon
        bool insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q){

        //determines if at goal within _epsilon distance. 
        bool atGoal(const amp::Problem2D& problem, , const Eigen::Vector2d& x);

        //determines if points are within _epsilon of eachother
        bool atPoint(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);

        // returns euclidian distance to goal
        double distToGoal(const amp::Problem2D& problem, x);
    private:
        // Add any member variables here...
        double _epsilon;
};