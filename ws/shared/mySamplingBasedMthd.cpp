#include "mySamplingBasedMthd.h"

void mySamplingBasedMthd::getData(std::shared_ptr<amp::Graph<double>>& g_ptr, std::map<amp::Node, Eigen::Vector2d>& m){
    if(!_save_data){
        std::cout<<"getData() called but _save_data set to false.\n";
        g_ptr = _graph_ptr; 
        m[0] = Eigen::Vector2d();
        return;
    }
    
    g_ptr = _graph_ptr; 

    {int i=0;
    for(auto p : _node_locs){
        m[i] = p;
    i++;}}
    return;
}


void mySamplingBasedMthd::smoothPath(const amp::Problem2D& prob, amp::Path2D& path){
    double dist;
    int len_start = path.waypoints.size();
    if(len_start>2){
        int i,j;
        for(int k=0; k<len_start*2; k++){
            int len = path.waypoints.size();
            // std::cout<<"one iteration\n";
            i = amp::RNG::randi(0,len);
            j = amp::RNG::randi(0,len);
            if(abs(j-i)<=1)
                continue;

            Eigen::Vector2d p1 = path.waypoints[i];
            Eigen::Vector2d p2 = path.waypoints[j];
            dist = (p1-p2).norm();
            if(H::freeBtwPointsLine(prob, p1, p2)){
                //remove nodes between i and j
                if(j<i){
                    int temp = i;
                    i = j;
                    j = temp;
                }
                // std::cout<<"Erasing from ["<<i<<", "<<j<<") len: "<<path.waypoints.size()<<"\n";
                path.waypoints.erase(path.waypoints.begin()+i+1,path.waypoints.begin()+j);                  ;
            }
        }
    }
    return;
}