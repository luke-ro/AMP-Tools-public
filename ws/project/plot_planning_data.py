import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
import json

g = 9.81

ARM_LEN = 0.15 #m
M_ARM = 0.05 #kg
M_MOTOR = 0.05 #kg
M = M_ARM + M_MOTOR

#    MOI of motors           MOI of structure     
Iy = 2*M_ARM*ARM_LEN**2 + (M_ARM*(2*ARM_LEN)**2)/12
def rotate(ang,vec):
    R = np.array([[np.cos(ang), -np.sin(ang)],
         [np.sin(ang), np.cos(ang)]])
    return R@vec

def plotquad(ax,state):
    motor = rotate(-state[2],[0.5,0]) 
    x_pts = [state[0]-motor[0], state[0]+motor[0]]
    y_pts = np.array([state[1]-motor[1], state[1]+motor[1]])
    return ax.plot(x_pts,-y_pts,color="steelblue")

def plotquad_data(state):
    motor = rotate(-state[2],[0.5,0]) 
    x_pts = [state[0]-motor[0], state[0]+motor[0]]
    y_pts = np.array([state[1]-motor[1], state[1]+motor[1]])
    return x_pts,y_pts

def plotquad_traj(ax,traj):
    for p in traj:
        plotquad(ax,p)

        vel_e = rotate(-p[2],p[3:5])
        if(np.linalg.norm(vel_e)>0):
            plt.arrow(p[0],-p[1],vel_e[0]*0.1,-vel_e[1]*0.1)

def plot_trajectory(ax,traj):
    # ax.plot(traj[:,0],traj[:,1])
    plotquad_traj(ax,traj)

def plotInstant(ax,y,particles,surface,xlim,ylim):
    x_quad,y_quad = plotquad_data(y)
    ax.plot(x_quad,y_quad,label="Quadcopter",color="r")[0]
    ax.scatter(particles[:,0],particles[:,1],label="Particles",s=1)

    x_sur = np.linspace(xlim[0],xlim[1],100)
    y_sur = [surface(val) for val in x_sur]
    ax.plot(x_sur,y_sur,label="Ground",color="g")
    ax.fill_between(x_sur,y_sur,np.ones(100)*ylim[1],color="g",alpha = 0.5)

    ax.invert_yaxis()
    ax.set(xlim=xlim,ylim=ylim,xlabel="x [m]",ylabel="y [m]")
    return ax


def animate_traj(traj,buffer=10,frame_time=30):
    # https://www.geeksforgeeks.org/using-matplotlib-for-animations/
    fig,ax = plt.subplots()
    x_min = np.min(traj[:,0])-buffer
    x_max = np.max(traj[:,0])+buffer
    z_min = np.min(traj[:,1])-buffer
    z_max = np.max(traj[:,1])+buffer

    x_quad,y_quad = plotquad_data(traj[0,:])
    line2 = ax.plot(x_quad,y_quad,label="Quadcopter",color="r")[0]

    ax.set(xlim=[x_min,x_max],ylim=[z_min,z_max],xlabel="x [m]",ylabel="y [m]")
    ax.invert_yaxis()
    ax.legend(loc="lower right")

    def update(i):

        x_quad,y_quad = plotquad_data(traj[i,:])
        line2.set_xdata(x_quad)
        line2.set_ydata(y_quad)

        return(line2)
    
    ani = FuncAnimation(fig=fig, func=update, frames = len(traj),interval=frame_time)
    return ani
    
if __name__ == "__main__":
    f = open("/home/user/repos/AMP-Tools-public/quad_planning_output.txt")
    data = json.load(f)
    traj1 = np.array(data["0"]["trajectory"])
    anim = animate_traj(traj=traj1)
    plt.show()