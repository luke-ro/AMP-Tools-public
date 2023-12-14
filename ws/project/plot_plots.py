import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

if __name__ == "__main__":
    epsilon_data = genfromtxt(r"/home/user/repos/AMP-Tools-public/testing_output_50_chang_eps.csv",delimiter=",")
    # print(epsilon_data)

    ## CHanging epsilon for a single agent

    fig,ax = plt.subplots()
    # quants = [(0.25,0.75) for _ in range(4)]
    ax.violinplot(epsilon_data.T, showmeans=False, showmedians=True)
    ax.yaxis.grid(True)
    # ax.set_title("")
    ax.set_ylabel("Time [s]")
    ax.set_xticks([1,2,3,4],["A","B","C","D"])
    plt.show()