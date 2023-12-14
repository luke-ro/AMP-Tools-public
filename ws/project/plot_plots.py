import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

if __name__ == "__main__":
    epsilon_data = genfromtxt(r"/home/user/repos/AMP-Tools-public/testing_output_50_chang_eps.csv",delimiter=",")
    epsilon_successes = [25,44,33,22]
    # print(epsilon_data)


    #fonts
    label_sz = 14
    fig_sz = (4,3)

    ## CHanging epsilon for a single agent
    labels = ["A","B","C","D"]
    fig,ax = plt.subplots(figsize=fig_sz)
    # quants = [(0.25,0.75) for _ in range(4)]
    ax.violinplot(epsilon_data.T, showmeans=False, showmedians=True)
    ax.yaxis.grid(True)
    # ax.set_title("")
    ax.set_ylabel("Time [s]",fontsize=label_sz)
    ax.set_xticks([1,2,3,4],labels)
    fig.tight_layout()

    fig,ax = plt.subplots(figsize=fig_sz)
    ax.yaxis.grid(True)
    ax.bar(labels,epsilon_successes,zorder=3)
    # ax.set_xlabel("Test Code",fontsize=label_sz)
    ax.set_ylabel("Number of Successes",fontsize=label_sz)
    # ax.legend(fontsize=label_sz)
    fig.tight_layout()


    ## multi agent data ws2 
    ma_ws2_data = genfromtxt(r"/home/user/repos/AMP-Tools-public/testing_output_multiagent_ws2.csv",delimiter=",")
    ma_ws2_succ = [46,23,8,5]

    fig,ax = plt.subplots(figsize=(4,3.3))
    # quants = [(0.25,0.75) for _ in range(4)]
    ax.violinplot(ma_ws2_data.T, showmeans=False, showmedians=True)
    ax.yaxis.grid(True)
    # ax.set_title("")
    ax.set_ylabel("Time [s]",fontsize=label_sz)
    ax.set_xticks([1,2,3,4],[1,2,3,4])
    ax.set_xlabel("Number of Agents",fontsize=label_sz)
    fig.tight_layout()

    fig,ax = plt.subplots(figsize=(4,3.3))
    ax.yaxis.grid(True)
    ax.bar([1,2,3,4],ma_ws2_succ,zorder=3)
    ax.set_xlabel("Number of Agents",fontsize=label_sz)
    ax.set_ylabel("Number of Successes",fontsize=label_sz)
    # ax.legend(fontsize=label_sz)
    fig.tight_layout()

    plt.show()