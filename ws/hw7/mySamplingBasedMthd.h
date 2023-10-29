#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include "myAStar.h"
#include "Helpers.h"

class mySamplingBasedMthd{
    public:

    mySamplingBasedMthd(int N=1000, double r=1.0, bool smoothing=true, bool save=false):
        _N_MAX(N),
        _neigh_radius(r),
        _smoothing(smoothing),
        _save_data(save){}


    void getData(std::shared_ptr<amp::Graph<double>>& g_ptr, std::map<amp::Node, Eigen::Vector2d>& m);
    void smoothPath(const amp::Problem2D& prob, amp::Path2D& path);



    std::vector<Eigen::Vector2d> _node_locs;
    // amp::Graph<double> _graph;
    std::shared_ptr<amp::Graph<double>> _graph_ptr;

    const int _N_MAX;
    const double _neigh_radius;
    const bool _smoothing;
    bool _save_data;
};