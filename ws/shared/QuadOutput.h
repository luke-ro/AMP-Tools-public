#include <fstream>
#include <iostream>
#include <string>
#include "QuadAgentProperties.h"

namespace QuadOutput{
    static bool writeToFile(QuadAgentsTrajectories trajectories, std::vector<std::vector<QuadControl>> controls);
    static bool writeTrajectoryJSON(std::ofstream& myfile, QuadAgentTrajectory traj);
    static bool writeControlsJSON(std::ofstream& myfile, std::vector<QuadControl> cont);
}

static bool QuadOutput::writeToFile(QuadAgentsTrajectories trajectories, std::vector<std::vector<QuadControl>> controls){
    int num_agents = trajectories.size();
    
    std::string agt_str = "agent_";
    
    std::ofstream myfile;
    myfile.open("/home/user/repos/AMP-Tools-public/quad_planning_output.txt");
    myfile<<"{\n";
    
    for(int i=0; i<num_agents; i++){
        myfile<<"\n\"" + std::to_string(i) + "\":\n";
        myfile<<"\t{\n";

        // myfile<<"\t\t{\n";
        myfile<<"\t\"trajectory\":";
        QuadOutput::writeTrajectoryJSON(myfile,trajectories[i]);
        myfile<<",\n\t\"controls\":";
        QuadOutput::writeControlsJSON(myfile,controls[i]);
        myfile<<"\n\t}";
        if(i!=num_agents-1) myfile<<",";

    }
    
    myfile<<"\n}";
    myfile.close();

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