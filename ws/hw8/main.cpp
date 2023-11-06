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
#include "myCSpace2d.h"
#include "Helpers.h"

const bool RUN_Q1 = true;
const bool RUN_MISC = false;
const bool RUN_GRADER = false;

int main(int argc, char** argv){
    
    if(RUN_Q1){
    if(RUN_Q1){
        amp::MultiAgentPath2D res;
        amp::Environment2D hw8_env1 = amp::HW8::getWorkspace1();
        // std::vector<amp::CircularAgentProperties> agent_properties;
        // amp::CircularAgentProperties.r

        std::vector<double> data_sizes;
        std::vector<double> data_times;
        std::list<std::vector<double>> data;

        int num_agents = 2;
        
            amp::MultiAgentProblem2D hw8_ws1 = amp::HW8::getWorkspace1(num_agents);
            for(int i=0; i<100; i++){
                std::cout<<i<<"\n";
                myCentMultiRRT centRRT(7500, 0.5, 0.05, 0.25);

                // start timer
                auto start = std::chrono::high_resolution_clock::now();

                //run the iteration of RRT
                res = centRRT.plan(hw8_ws1);

                // end timer
                auto end = std::chrono::high_resolution_clock::now();

                std::chrono::duration<double> duration = end - start;

                int size = centRRT.getTreeSize();

                data_times.push_back(double(duration.count()));
                data_sizes.push_back(size);

            }

        data.push_back(data_times);
        data.push_back(data_sizes);
        std::vector<std::string> labels = {"Times","Sizes"};
        amp::Visualizer::makeBoxPlot(data,labels,"m="+std::to_string(num_agents),"","");

        amp::Visualizer::makeFigure(hw8_ws1, res);

    }

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

    }

    if(RUN_GRADER){

    }


    if(!RUN_GRADER)
        amp::Visualizer::showFigures();

    return 0;
}