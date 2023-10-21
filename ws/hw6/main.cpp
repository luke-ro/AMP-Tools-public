// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

#include "time.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW5.h"
#include "hw/HW6.h"

#include "myWFPoint.h"
#include "myWFManip.h"
#include "CSpace2D.h"
#include "Helpers.h"
#include "MyGridCon.h"
#include "Arm2L.h"
#include "myAStar.h"

int main(int argc, char** argv) {

        //test wavefron
        myWFPoint wf;
        // srand(time(NULL));
        // int seed = time(NULL); //failed 1697687252
        int seed = 1697687252;
        std::cout<<"seed: "<< seed<<"\n";
        amp::Problem2D ws =  amp::HW2::getWorkspace2();
        auto grid_ptr = wf.constructDiscretizedWorkspace(ws);
        amp::Path2D wf_path;
        wf_path =  wf.planInCSpace(ws.q_init, ws.q_goal, *grid_ptr);
        std::cout<<"Path distance for wavefront on HW2 WS2: "<< wf_path.length()<<"\n";

        amp::Problem2D rand_prob;
        amp::Path2D rand_path;
        amp::HW6::generateAndCheck(wf,rand_path, rand_prob,true,seed);

        //make plots q1
        amp::Visualizer::makeFigure(*grid_ptr,wf_path);
        amp::Visualizer::makeFigure(ws,wf_path);
        amp::Visualizer::makeFigure(*grid_ptr);
        amp::Visualizer::makeFigure(rand_prob, rand_path);
        // amp::Visualizer::showFigures();
    



            /*** 2 ***/
        Eigen::Vector2d base {0,0};
        std::vector<double> lengths_3 = {1.0, 1.0};
        Arm2L manip_3(base, lengths_3); 
        // CSpace2D gridcon(0, 2*3.1415, 0, 2*3.1415, 100, 100);


        // make arm
        std::vector<double> link_lengths = {1.0, 1.0};
        Arm2L lm(link_lengths);

        // get env
        // amp::Environment2D arm_env = amp::HW4::getEx3Workspace1();
        amp::Problem2D arm_prob = amp::HW6::getHW4Problem2();

        // figure out arm q_inti and q_goal
        Eigen::Vector2d end_eff_init {-2,0};
        Eigen::Vector2d end_eff_goal {2,0};
        amp::ManipulatorState q_init = lm.getConfigurationFromIK(end_eff_init);
        amp::ManipulatorState q_goal = lm.getConfigurationFromIK(end_eff_goal);

        // Plan through Arm CSPace
        // auto grid_constructor = std::make_unique<MyGridCon>();
        // std::unique_ptr<MyGridCon> grid_constructor (new MyGridCon());
        MyGridCon grid_constructor(25,25);
        myWFManip wf_manip(grid_constructor);

        // generate and plan in the cspace
        // auto cspace_manip = gridcon.genCSpace(manip_3,arm_env);
        auto cspace_manip = grid_constructor.construct(manip_3,arm_prob);
        amp::Path2D arm_path =  wf_manip.planInCSpace(q_init, q_goal, *cspace_manip);

        //Random 
        amp::Path2D path_rand; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = amp::HW6::generateAndCheck(wf_manip,manip_3,path_rand,random_prob,collision_points);

        // plot
        amp::Visualizer::makeFigure(*cspace_manip,arm_path);
        amp::Visualizer::makeFigure(arm_prob,lm,arm_path);
        amp::Visualizer::makeFigure(random_prob,lm,path_rand);


    
        //my astar
        myAStar as;
        amp::AStar::GraphSearchResult astar_res = as.search(amp::HW6::getEx3SPP(), amp::HW6::getEx3Heuristic());
        int sz = astar_res.node_path.size();
        for(int i=0; i<sz; i++){
            std::cout<<astar_res.node_path.back()<<"\n";
            astar_res.node_path.pop_back();
        }
        
        amp::Visualizer::showFigures();

    // static int grade(amp::PointMotionPlanner2D& , amp::LinkManipulatorMotionPlanner2D&, amp::AStar&, email, argc, argv);
    amp::HW6::grade( wf, wf_manip, as, "luke.roberson@colorado.edu", argc, argv);
    return 0;
}