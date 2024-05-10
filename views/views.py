import numpy as np
import matplotlib.pyplot as plt
import os
from controller.robot import *
from const import const

def plot_single_robot(robot: Robot):
    data = robot.lst_hisPos

    startPos = data[0]
    goalPos = robot.np_goal

    X = [point[0] for point in data]
    Y = [point[1] for point in data]

    length = len(data)

    plt.plot(X, Y, label='Path')
    # plt.scatter(startPos[0], startPos[1], label='Start Position', marker='8', s=50)
    plt.scatter(goalPos[0], goalPos[1], label='Goal Position', marker='*', s=100, color='red', zorder=length+1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-1, const.OFFSET*2 + 1)
    plt.ylim(-1, const.OFFSET*2 + 1)
    plt.title('Robot Path')
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_multiple_robots(robots: list['Robot']):
    currentPath = os.getcwd()
    for robot in robots:
        
        data = robot.lst_hisPos

        # startPos = data[0]
        goalPos = robot.np_goal

        X = [point[0] for point in data]
        Y = [point[1] for point in data]

        length = len(data)

        plt.plot(X, Y, label='Path')
        # plt.scatter(startPos[0], startPos[1], marker='8', s=50)
        plt.scatter(goalPos[0], goalPos[1], marker='*', s=100, zorder=length+1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xlim(-1, const.OFFSET*2 + 1)
        plt.ylim(-1, const.OFFSET*2 + 1)
        plt.title('Robot Path')
        plt.grid(True)
    plt.savefig(currentPath + '/output/result.png')
    plt.close()

def plot_step_robots(robots: list['Robot']):
    currentPath = os.getcwd()
    iteration = len(robots[0].lst_hisPos)
    for i in range(iteration):
        for robot in robots:
            data = robot.lst_hisPos

            # startPos = data[0]
            goalPos = robot.np_goal

            X = [point[0] for point in data]
            Y = [point[1] for point in data]

            length = len(data)

            plt.plot(X[:i], Y[:i])
            # plt.scatter(startPos[0], startPos[1], marker='8', s=50)
            plt.scatter(goalPos[0], goalPos[1], marker='*', s=100, zorder=length+1)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.xlim(-1, const.OFFSET*2 + 1)
            plt.ylim(-1, const.OFFSET*2 + 1)
            plt.title('Robot Path')
            plt.grid(True)
        plt.savefig(currentPath + f'/output/step/result{i}.png')
        plt.close()

