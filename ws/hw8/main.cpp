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
#include "hw/HW8.h"

#include "myCentMultiRRT.h"
#include "myDecenMultiRRT.h"
#include "myCSpace2d.h"
#include "Helpers.h"

const bool RUN_Q1 = false;
const bool RUN_Q2 = false;
const bool RUN_MISC = true;
const bool RUN_GRADER = false;

double mean(std::vector<double> vec){
    double sum=0;
    for(int i=0; i<vec.size(); i++){
        sum+=vec[i];
    }
    return sum/vec.size();
}

int main(int argc, char** argv){
    
    if(RUN_Q1){
        amp::MultiAgentPath2D res;
        amp::Environment2D hw8_env1 = amp::HW8::getWorkspace1();
        // std::vector<amp::CircularAgentProperties> agent_properties;
        // amp::CircularAgentProperties.r

        std::vector<std::vector<double>> data_sizes;
        std::vector<std::vector<double>> data_times;
 

        
        for(int num_agents=2; num_agents<=6; num_agents++){
            std::vector<double> sizes;
            std::vector<double> times;

            amp::MultiAgentProblem2D hw8_ws1 = amp::HW8::getWorkspace1(num_agents);
            int valid_count=0;
            for(int i=0; i<100; i++){
        
                std::cout<<num_agents<<"\n";
                myCentMultiRRT centRRT(7500, 0.5, 0.05, 0.25);

                // start timer
                auto start = std::chrono::high_resolution_clock::now();

                //run the iteration of RRT
                res = centRRT.plan(hw8_ws1);

                // end timer
                auto end = std::chrono::high_resolution_clock::now();

                std::chrono::duration<double> duration = end - start;

                int size = centRRT.getTreeSize();

                bool valid = amp::HW8::check(res, hw8_ws1, false);
                valid_count+=int(valid);

                // if(valid){
                    times.push_back(double(duration.count()));
                    sizes.push_back(size);
                // }

            }
            std::cout<<"Valid: "<<valid_count<<"\n";
            data_times.push_back(times);
            data_sizes.push_back(sizes);
        }

        std::list<std::vector<double>> data_times_list(data_times.begin(),data_times.end());
        std::list<std::vector<double>> data_sizes_list(data_sizes.begin(),data_sizes.end());
        std::vector<std::string> labels = {"m=2","m=3","m=4","m=5","m=6"};
        amp::Visualizer::makeBoxPlot(data_times_list,labels,"Times","","");
        amp::Visualizer::makeBoxPlot(data_sizes_list,labels,"Sizes","","");

        std::vector<double> time_means(5);
        std::vector<double> size_means(5);

        for(int i=0; i<5; i++){
            time_means[i] = mean(data_times[i]);
            size_means[i] = mean(data_sizes[i]);
        }
        amp::Visualizer::makeBarGraph(time_means,labels,"Average Times","","");
        amp::Visualizer::makeBarGraph(size_means,labels,"Average Sizes","","");



    }

    if(RUN_Q2){
        amp::MultiAgentPath2D res;
        amp::Environment2D hw8_env1 = amp::HW8::getWorkspace1();
        // std::vector<amp::CircularAgentProperties> agent_properties;
        // amp::CircularAgentProperties.r

        std::vector<std::vector<double>> data_sizes;
        std::vector<std::vector<double>> data_times;
 

        
        for(int num_agents=2; num_agents<=2; num_agents++){
            std::vector<double> sizes;
            std::vector<double> times;

            amp::MultiAgentProblem2D hw8_ws1 = amp::HW8::getWorkspace1(num_agents);
            int valid_count=0;
            for(int i=0; i<1; i++){
        
                std::cout<<num_agents<<"\n";
                myDecenMultiRRT centRRT(7500, 0.5, 0.05, 0.25);

                // start timer
                auto start = std::chrono::high_resolution_clock::now();

                //run the iteration of RRT
                res = centRRT.plan(hw8_ws1);

                // end timer
                auto end = std::chrono::high_resolution_clock::now();

                std::chrono::duration<double> duration = end - start;

                int size = centRRT.getTreeSize();

                bool valid = amp::HW8::check(res, hw8_ws1, false);
                valid_count+=int(valid);

                // if(valid){
                    times.push_back(double(duration.count()));
                    sizes.push_back(size);
                // }

            }
            std::cout<<"Valid: "<<valid_count<<"\n";
            data_times.push_back(times);
            data_sizes.push_back(sizes);
        }

        std::list<std::vector<double>> data_times_list(data_times.begin(),data_times.end());
        std::list<std::vector<double>> data_sizes_list(data_sizes.begin(),data_sizes.end());
        std::vector<std::string> labels = {"m=2","m=3","m=4","m=5","m=6"};
        amp::Visualizer::makeBoxPlot(data_times_list,labels,"Times","","");
        amp::Visualizer::makeBoxPlot(data_sizes_list,labels,"Sizes","","");

        std::vector<double> time_means(5);
        std::vector<double> size_means(5);

        for(int i=0; i<5; i++){
            time_means[i] = mean(data_times[i]);
            size_means[i] = mean(data_sizes[i]);
        }
        amp::Visualizer::makeBarGraph(time_means,labels,"Average Times","","");
        amp::Visualizer::makeBarGraph(size_means,labels,"Average Sizes","","");
    }

    if(RUN_MISC){
        // amp::MultiAgentProblem2D hw8_ws1 = amp::HW8::getWorkspace1();
        // amp::MultiAgentPath2D test;
        // test.numAgents = 2;
        // std::vector<amp::Path2D> paths;
        // amp::Path2D path1;
        // path1.waypoints.push_back(hw8_ws1.agent_properties[0].q_init);
        // path1.waypoints.push_back(hw8_ws1.agent_properties[0].q_goal);
        // amp::Path2D path2;
        // path1.waypoints.push_back(hw8_ws1.agent_properties[0].q_init);
        // path1.waypoints.push_back(hw8_ws1.agent_properties[0].q_goal);
        // test.agent_paths = 
        // amp::HW8::check(res,hw8_ws1);
        myDecenMultiRRT RRT(7500, 0.5, 0.05, 0.25);
        amp::MultiAgentProblem2D hw8_ws1 = amp::HW8::getWorkspace1(2);
        amp::MultiAgentPath2D res;
        // {int i=0;
        // do{
        //     std::cout<<i++<<"\n";
        //     res = centRRT.plan(hw8_ws1);
        // }while(!amp::HW8::check(res,hw8_ws1));}
        res = RRT.plan(hw8_ws1);
        amp::HW8::check(res,hw8_ws1);
        amp::Visualizer::makeFigure(hw8_ws1, res);

    }

    if(RUN_GRADER){

    }


    if(!RUN_GRADER)
        amp::Visualizer::showFigures();

    return 0;
}