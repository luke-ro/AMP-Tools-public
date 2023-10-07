#include "Arm2L.h"
#include "Rotate.h"

Arm2L::Arm2L(const std::vector<double>& link_lengths) 
        : LinkManipulator2D(link_lengths)
        {}

Arm2L::Arm2L(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths) 
        : LinkManipulator2D(base_location, link_lengths)
        {}


Eigen::Vector2d Arm2L::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{
    Eigen::Vector2d loc = m_base_location;

    if(joint_index>state.size()){
        return loc;
        }

    double angle = 0;
    // Go through links to get nth joint location
    for(int i=0; i<joint_index; i++){
        Eigen::Vector2d temp = {m_link_lengths[i],0};
        
        // {m_link_lengths[i]*cos(state[i]),m_link_lengths[i]*sin(state[i])}

        angle+=state[i];
        loc += Rotate::rotatePoint(temp, angle, Eigen::Vector2d(0,0));
    }

    return loc;
}


amp::ManipulatorState Arm2L::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    
    // check if there is a solution
    double sum=0;
    for(auto L : m_link_lengths) sum+=L;
    if(sum < sqrt(pow(m_base_location[0]-end_effector_location[0],2)+pow(m_base_location[1]-end_effector_location[1],2))){
        std::cout<<"Not posible to reach point with link lengths"<<"\n";
        return amp::ManipulatorState ();
    }
    
    //Find solution circle
    Eigen::Vector2d circ_x_lims = {end_effector_location[0]-m_link_lengths[2], end_effector_location[0]+m_link_lengths[2]};
    Eigen::Vector2d circ_y_lims = {end_effector_location[1]-m_link_lengths[2], end_effector_location[1]+m_link_lengths[2]};
    double circ_r2 = pow(circ_x_lims[1]-circ_x_lims[0],2);

    //lambda func for the solution circle
    auto f_sol_circ = [end_effector_location,circ_r2](double x){return sqrt(circ_r2-pow(x-end_effector_location[0],2))+end_effector_location[1];}; 

    // this should give the point 
    double angle = Rotate::ang(end_effector_location, m_base_location);
    Eigen::Vector2d j2 {end_effector_location[0]+m_link_lengths[2]*cos(angle),
                        end_effector_location[1]+m_link_lengths[2]*sin(angle)};

    // 2 link problem
    double l1 = m_link_lengths[0];
    double l2 = m_link_lengths[1];
    double ctheta2 = 1.0/(2.0*l1*l2)*(((j2[0]*j2[0])+(j2[1]*j2[1]))-((l1*l1) + (l2*l2)));
    double stheta2 = sqrt(1-(ctheta2*ctheta2));
    double ctheta1 = 1/((j2[0]*j2[0])+(j2[1]*j2[1]))*(j2[0]*(l1+(l2*ctheta2))+(j2[1]*l2*sqrt(1-(ctheta2))));
    double stheta1 = 1/((j2[0]*j2[0])+(j2[1]*j2[1]))*(j2[1]*(l1+(l2*ctheta2))-(j2[0]*l2*sqrt(1-(ctheta2))));

    std::vector<double> angs;
    angs.push_back(atan2(stheta1,ctheta1));
    angs.push_back(atan2(stheta2,ctheta2));
    angs.push_back(Rotate::ang(j2,end_effector_location)-angs[0]-angs[1]);

    std::vector<double> to_ret;
    for (int i=0;  i<m_link_lengths.size(); i++){
        to_ret.push_back(angs[i]);
    }


    return to_ret;
}
