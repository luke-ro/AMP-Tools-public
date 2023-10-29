#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include "myAStar.h"
#include "Helpers.h"

class myPRM2D : public amp::PRM2D{
    public:



    myPRM2D(int N=1000, double r=1.0, bool smoothing=true, bool save=false):
        _N_MAX(N),
        _neigh_radius(r),
        _smoothing(smoothing),
        _save_data(save){}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    void getPRMData(std::shared_ptr<amp::Graph<double>>& g_ptr, std::map<amp::Node, Eigen::Vector2d>& m);
  

    private:

    std::vector<Eigen::Vector2d> _node_locs;
    // amp::Graph<double> _graph;
    std::shared_ptr<amp::Graph<double>> _graph_ptr;


    const int _N_MAX;
    const double _neigh_radius;
    const bool _smoothing;
    bool _save_data;
};