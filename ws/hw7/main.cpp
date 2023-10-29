// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"
#include <chrono>

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW7.h"

#include "myPRM2D.h"

int main(){
    // int t = time(NULL);
    int t = 1698538574;
    amp::RNG::seed(t);
    std::cout<<"seed: "<<t<<"\n";

    srand(0);



    amp::Problem2D prob_hw5_ws1 = amp::HW5::getWorkspace1();
    amp::Problem2D prob_hw2_ws1 = amp::HW2::getWorkspace1();


    std::vector<std::pair<int,double>> nr_sets(8);
    nr_sets[0] = std::pair<int,double>(200,0.5);
    nr_sets[0] = std::pair<int,double>(200,0.5);
    nr_sets[1] = std::pair<int,double>(200,1.0);
    nr_sets[2] = std::pair<int,double>(200,1.5);
    nr_sets[3] = std::pair<int,double>(200,2.0);
    nr_sets[4] = std::pair<int,double>(500,0.5);
    nr_sets[5] = std::pair<int,double>(500,1.0);
    nr_sets[6] = std::pair<int,double>(500,1.5);
    nr_sets[7] = std::pair<int,double>(500,2.0);

    std::vector<std::string> labels(8);
    labels[0] = "n=200, r=0.5";
    labels[1] = "n=200, r=1.0";
    labels[2] = "n=200, r=1.5";
    labels[3] = "n=200, r=2.0";
    labels[4] = "n=500, r=0.5";
    labels[5] = "n=500, r=1.0";
    labels[6] = "n=500, r=1.5";
    labels[7] = "n=500, r=2.0";

    
    std::list<std::vector<double>> data_sols;
    std::list<std::vector<double>> data_length;
    std::list<std::vector<double>> data_time;

    amp::Path2D path1;

    bool smooth = true;
    // for(auto nr : nr_sets){
    //     std::vector<double> num_valid={0.0};
    //     std::vector<double> lengths;
    //     std::vector<double> times;
    //     for(int i=0; i<5; i++){
    //         std::vector<double> data;
    //         myPRM2D prm(nr.first,nr.second,smooth);

    //         // start time
    //         auto start = std::chrono::high_resolution_clock::now();
    //         path1 = prm.plan(prob_hw5_ws1);
    //         auto end = std::chrono::high_resolution_clock::now();
    //         // end time

    //         // get time diff
    //         std::chrono::duration<double> duration = end - start;
            
    //         bool valid = amp::HW7::check(path1,prob_hw5_ws1);

    //         num_valid[0] += double(valid);
    //         lengths.push_back(path1.length());
    //         times.push_back(double(duration.count()));
    //     }


    //     data_sols.push_back(num_valid);
    //     data_length.push_back(lengths);
    //     data_time.push_back(times);
    // }

    // amp::Visualizer::makeBoxPlot(data_length,labels,"Lengths","","");
    // amp::Visualizer::makeBoxPlot(data_time,labels,"Times","","");
    // amp::Visualizer::makeBoxPlot(data_sols,labels,"Valid Solutions","","");

// static void makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, 
//                                 const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());
    myPRM2D prm(600,1.0,true,true);
    path1 = prm.plan(prob_hw2_ws1);
    amp::Visualizer::makeFigure(prob_hw2_ws1, path1);

    std::shared_ptr<amp::Graph<double>> ptr;
    std::map<amp::Node, Eigen::Vector2d> node_map;
    prm.getPRMData(ptr,node_map);
    amp::Visualizer::makeFigure(prob_hw2_ws1,*ptr,node_map);

    amp::Visualizer::showFigures();
    return 0;
}