import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

if __name__ == "__main__":
    epsilon_data = genfromtxt(r"/home/user/repos/AMP-Tools-public/testing_output_50_chang_eps.csv",delimiter=",")
    epsilon_successes = [25,44,33,22]
    # print(epsilon_data)

    ## CHanging epsilon for a single agent
    labels = ["A","B","C","D"]
    fig,ax = plt.subplots(figsize=(4,3))
    # quants = [(0.25,0.75) for _ in range(4)]
    ax.violinplot(epsilon_data.T, showmeans=False, showmedians=True)
    ax.yaxis.grid(True)
    # ax.set_title("")
    ax.set_ylabel("Time [s]")
    ax.set_xticks([1,2,3,4],labels)

    fig,ax = plt.subplots(figsize=(4,3))
    ax.yaxis.grid(True)
    ax.bar(labels,epsilon_successes,zorder=3)
    ax.set_xlabel("Test Code",fontsize=15)
    ax.set_ylabel("Number of Successes")
    plt.show()