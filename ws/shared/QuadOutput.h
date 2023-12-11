#include <fstream>
#include <iostream>
#include <string>
#include "QuadAgentProperties.h"

namespace QuadOutput{
    static bool writeToFile(QuadAgentProblem prob, std::vector<QuadAgentProperties> agents, QuadAgentsTrajectories trajectories, std::vector<std::vector<QuadControl>> controls);
    static bool writeTrajectoryJSON(std::ofstream& myfile, QuadAgentTrajectory traj);
    static bool writeControlsJSON(std::ofstream& myfile, std::vector<QuadControl> cont);
    static bool writeObstaclesJSON(std::ofstream& myfile, std::vector<amp::Polygon> obstacles);
    static bool writeStateJSON(std::ofstream& myfile, QuadState x);

}

static bool QuadOutput::writeToFile(QuadAgentProblem prob, std::vector<QuadAgentProperties> agents, QuadAgentsTrajectories trajectories, std::vector<std::vector<QuadControl>> controls){
    int num_agents = trajectories.size();
    
    std::string agt_str = "agent_";
    
    std::ofstream myfile;
    myfile.open("/home/user/repos/AMP-Tools-public/quad_planning_output.txt");
    myfile<<"{\n";
    
    myfile<<"\"agents\":\n{";

    for(int i=0; i<num_agents; i++){
        myfile<<"\n\"" + std::to_string(i) + "\":\n";
        myfile<<"\t{\n";

        // myfile<<"\t\t{\n";
        myfile<<"\t\"trajectory\":";
        QuadOutput::writeTrajectoryJSON(myfile,trajectories[i]);
        myfile<<",\n";
        
        myfile<<"\t\"controls\":";
        QuadOutput::writeControlsJSON(myfile,controls[i]);
        myfile<<",\n";
        
        myfile<<"\t\"q_init\":"; 
        QuadOutput::writeStateJSON(myfile,agents[i].q_init);
        myfile<<",\n";

        myfile<<"\t\"q_goal\":"; 
        QuadOutput::writeStateJSON(myfile,agents[i].q_goal);

        myfile<<"\n\t}";
        if(i!=num_agents-1) 
            myfile<<",";

    }
    
    myfile<<"\n},\n";


    myfile<<"\"obstacles\":";
    QuadOutput::writeObstaclesJSON(myfile, prob.env.obstacles);


    // end bracket
    myfile<<"\n}";

    myfile.close();

    return true;
}


static bool QuadOutput::writeObstaclesJSON(std::ofstream& myfile, std::vector<amp::Polygon> obstacles){
    myfile<<"[";
    for (int i=0; i<obstacles.size(); i++){
        myfile<<"[";
        for (int j=0; j<obstacles[i].verticesCCW().size(); j++){
            myfile<<"[";
            myfile<<obstacles[i].verticesCCW()[j][0];
            myfile<<", ";
            myfile<<obstacles[i].verticesCCW()[j][1];
            myfile<<"]";
            if(j!=obstacles[i].verticesCCW().size()-1) 
                myfile<<",";
        }
        myfile<<"]";
        if(i!=obstacles.size()-1) 
            myfile<<",";
    }
    myfile<<"]";
    return true;
}

static bool QuadOutput::writeStateJSON(std::ofstream& myfile, QuadState x){
    myfile<<"[";
    int i;
    for(i=0; i<5; i++)
        myfile<<std::to_string(x[i]) << ", ";

    myfile<<std::to_string(x[i]);
    myfile<<"]";

    return true;
}

static bool QuadOutput::writeTrajectoryJSON(std::ofstream& myfile, QuadAgentTrajectory traj){
    myfile<<"[";
    int i;
    if(traj.size()>0){
        for(i=0; i<traj.size()-1; i++){
            QuadState x = traj[i]; 
            myfile<<"[";
            for(int j=0; j<x.rows()-1; j++){
                myfile<<std::to_string(x(j))<<",";
            }
            myfile<<std::to_string(x(x.rows()-1));
            myfile<<"],";
        }

        QuadState x = traj[i]; 
        myfile<<"[";
        for(int j=0; j<x.rows()-1; j++){
            myfile<<std::to_string(x(j))<<",";
        }
        myfile<<std::to_string(x(x.rows()-1));
        myfile<<"]";
    }else{
        myfile<<"[]";
    }

    myfile<<"]";

    return true;

}

static bool QuadOutput::writeControlsJSON(std::ofstream& myfile, std::vector<QuadControl> cont){
    myfile<<"[";
    int i;
    if(cont.size()>0){
        for(i=0; i<cont.size()-1; i++){
            QuadControl x = cont[i]; 
            myfile<<"[";
            for(int j=0; j<x.rows()-1; j++){
                myfile<<std::to_string(x(j))<<",";
            }
            myfile<<std::to_string(x(x.rows()-1));
            myfile<<"],";
        }

        QuadControl x = cont[i]; 
        myfile<<"[";
        for(int j=0; j<x.rows()-1; j++){
            myfile<<std::to_string(x(j))<<",";
        }
        myfile<<std::to_string(x(x.rows()-1));
        myfile<<"]";
    }else{
        myfile<<"[]";
    }

    myfile<<"]";

    return true;

}