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
#include "Helpers.h"

const bool RUN_Q1A = false;
const bool RUN_Q1B = true;

int main(){
    // int t = time(NULL);
    int t = 1698538574;
    amp::RNG::seed(t);
    std::cout<<"seed: "<<t<<"\n";

    srand(0);

    amp::Problem2D prob_hw2_ws1 = amp::HW2::getWorkspace1();
    amp::Problem2D prob_hw2_ws2 = amp::HW2::getWorkspace2();
    amp::Problem2D prob_hw5_ws1 = amp::HW5::getWorkspace1();

    // std::vector<Eigen::Vector2d> locs_vec(5);
    // locs_vec[0] = Eigen::Vector2d(0,0);
    // locs_vec[1] = Eigen::Vector2d(1,0);
    // locs_vec[2] = Eigen::Vector2d(0,1);
    // locs_vec[3] = Eigen::Vector2d(-1,0);
    // locs_vec[4] = Eigen::Vector2d(0,-1);
    // for(auto loc : locs_vec)
    //     std::vector<amp::Node> neighbors = H::getNeighbors(locs_vec, loc, 2, 0);

    /*** Q1a ***/
    if(RUN_Q1A){
        
        prob_hw5_ws1.x_min = -1;
        prob_hw5_ws1.x_max = 11;
        prob_hw5_ws1.y_min = -3;
        prob_hw5_ws1.y_max = 3;

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

        bool smooth = false;
        for(auto nr : nr_sets){
            std::vector<double> num_valid={0.0};
            std::vector<double> lengths;
            std::vector<double> times;
            for(int i=0; i<100; i++){
                std::vector<double> data;
                myPRM2D prm(nr.first,nr.second,smooth);

                // start time
                auto start = std::chrono::high_resolution_clock::now();
                path1 = prm.plan(prob_hw5_ws1);
                auto end = std::chrono::high_resolution_clock::now();
                // end time

                // get time diff
                std::chrono::duration<double> duration = end - start;
                
                bool valid = amp::HW7::check(path1,prob_hw5_ws1);

                num_valid[0] += double(valid);
                if(valid) lengths.push_back(path1.length());
                times.push_back(double(duration.count()));
            }


            data_sols.push_back(num_valid);
            data_length.push_back(lengths);
            data_time.push_back(times);
        }
        if(smooth){
            amp::Visualizer::makeBoxPlot(data_length,labels,"Lengths {smoothing: on}","","");
            amp::Visualizer::makeBoxPlot(data_time,labels,"Times {smoothing: on}","","");
            amp::Visualizer::makeBoxPlot(data_sols,labels,"Valid Solutions {smoothing: on}","","");
        }else{
            amp::Visualizer::makeBoxPlot(data_length,labels,"Lengths {smoothing: off}","","");
            amp::Visualizer::makeBoxPlot(data_time,labels,"Times {smoothing: off}","","");
            amp::Visualizer::makeBoxPlot(data_sols,labels,"Valid Solutions {smoothing: off}","","");
        }
    }
    //END Q1A


    /*** Q1b ***/
    if(RUN_Q1B){

        std::vector<amp::Problem2D> probs1b(2);
        probs1b[0] = prob_hw2_ws1;
        probs1b[1] = prob_hw2_ws2;

        for(auto prob : probs1b){
            myPRM2D prm1b(200,2.0,false,true);
            amp::Path2D path1b = prm1b.plan(prob);
            amp::Visualizer::makeFigure(prob, path1b);
            std::cout<<"1b pathlength: "<<path1b.length()<<"\n";

            std::shared_ptr<amp::Graph<double>> ptr1b;
            std::map<amp::Node, Eigen::Vector2d> node_map1b;
            prm1b.getData(ptr1b,node_map1b);
            amp::Visualizer::makeFigure(prob,*ptr1b,node_map1b);
        }

        std::vector<std::pair<int,double>> nr_sets(6);
        nr_sets[0] = std::pair<int,double>(200,1.0);
        nr_sets[1] = std::pair<int,double>(200,2.0);
        nr_sets[2] = std::pair<int,double>(500,1.0);
        nr_sets[3] = std::pair<int,double>(500,2.0);
        nr_sets[4] = std::pair<int,double>(1000,1.0);
        nr_sets[5] = std::pair<int,double>(1000,2.0);

        std::vector<std::string> labels(6);
        labels[0] = "n=200, r=1.0";
        labels[1] = "n=200, r=2.0";
        labels[2] = "n=500, r=1.0";
        labels[3] = "n=500, r=2.0";
        labels[4] = "n=1000, r=1.0";
        labels[5] = "n=1000, r=2.0";


        
        std::list<std::vector<double>> data_sols;
        std::list<std::vector<double>> data_length;
        std::list<std::vector<double>> data_time;

        amp::Path2D path;

        bool smooth = false;
        amp::Problem2D prob = prob_hw2_ws1;
        for(auto nr : nr_sets){
            std::vector<double> num_valid={0.0};
            std::vector<double> lengths;
            std::vector<double> times;
            for(int i=0; i<100; i++){
                std::vector<double> data;
                myPRM2D prm(nr.first,nr.second,smooth);

                // start time
                auto start = std::chrono::high_resolution_clock::now();
                path = prm.plan(prob);
                auto end = std::chrono::high_resolution_clock::now();
                // end time

                // get time diff
                std::chrono::duration<double> duration = end - start;
                
                bool valid = amp::HW7::check(path,prob);

                num_valid[0] += double(valid);
                if(valid) lengths.push_back(path.length());
                times.push_back(double(duration.count()));
            }


            data_sols.push_back(num_valid);
            data_length.push_back(lengths);
            data_time.push_back(times);
        }
        if(smooth){
            amp::Visualizer::makeBoxPlot(data_length,labels,"Lengths {smoothing: on}","","");
            amp::Visualizer::makeBoxPlot(data_time,labels,"Times {smoothing: on}","","");
            amp::Visualizer::makeBoxPlot(data_sols,labels,"Valid Solutions {smoothing: on}","","");
        }else{
            amp::Visualizer::makeBoxPlot(data_length,labels,"Lengths {smoothing: off}","","");
            amp::Visualizer::makeBoxPlot(data_time,labels,"Times {smoothing: off}","","");
            amp::Visualizer::makeBoxPlot(data_sols,labels,"Valid Solutions {smoothing: off}","","");
        }

    }
    //END Q1B

// static void makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, 
//                                 const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());
    myPRM2D prm(600,1.0,true,true);
    amp::Path2D path = prm.plan(prob_hw2_ws1);
    amp::Visualizer::makeFigure(prob_hw2_ws1, path);

    std::shared_ptr<amp::Graph<double>> ptr;
    std::map<amp::Node, Eigen::Vector2d> node_map;
    prm.getData(ptr,node_map);
    amp::Visualizer::makeFigure(prob_hw2_ws1,*ptr,node_map);

    amp::Visualizer::showFigures();
    return 0;
}