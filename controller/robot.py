from const import const
import numpy as np
from numpy.linalg import norm

class Robot:
    def __init__(self, index: int, init_pos: np.array, init_goal: np.array):
        """
        Initialize the robot with its index, initial position, and goal position.

        Parameters:
        -----------
            index: int - index of the robot
            init_pos: np.array - initial position of the robot
            init_goal: np.array - goal position of the robot

        Returns:
        --------
            None
        """
        self.int_index: int = index             # index of the robot
        self.np_curPos: np.array = init_pos     # current position of the robot
        self.np_goal: np.array = init_goal      # goal position of the robot
        self.lst_hisPos: list['np.array'] = [self.np_curPos]  # history of the robot's position
    
    
    def update_pos(self, np_control: np.array, dt: float):
        """
        Update the position of the robot based on the control signal and the time step.
    
        Parameters:
        -----------
            np_control: np.array - control signal
            dt: float - time step
            
        Returns:
        --------
            None
        """
        self.np_curPos = self.np_curPos + np_control*dt
        self.lst_hisPos.append(self.np_curPos)
    
    def calc_control_signal(self, lst_robots):
        """
        Calculate the control signal for the robot. 

        Parameters:
        -----------
            lst_robots: list - list of robots

        Returns:
        --------
            np_control: np.array - control signal
        """
        # calculate the control signal for move to goal
        np_mtg = self.move_to_goal(self.np_goal)

        # calculate the control signal for avoid collision
        np_ac  = self.avoid_collision(lst_robots)

        # calculate the control signal for random walk
        np_rw  = self.random_walk()

        # calculate the total control signal
        np_control = np_mtg * const.W_MTG \
                    + np_ac * const.W_AC \
                    + np_rw * const.W_RW
        
        return np_control
    
    def move_to_goal(self, np_goalPos: np.array):
        """
        Create control signal to move the robot to the goal position.

        Parameters:
        -----------
            goal: np.array - goal position

        Returns:
        --------
            np_mtg: np.array - control signal
        """
        # calculate the relative position and distance between the current robot and the goal position
        rel_dis = self.calc_relative_pos(self.np_curPos, np_goalPos)
        dis     = self.calc_norm_dis(self.np_curPos, np_goalPos)

        # if the robot is already at the goal position, return zero vector
        if dis < const.E:
            return const.NP_ZERO
        
        # create control signal
        np_mtg = rel_dis/dis
        return np_mtg

    def avoid_collision(self, lst_robots: list['Robot']):
        """
        Create control signal to avoid collision with other robots.

        Parameters:
        -----------
            lst_robots: list - list of robots

        Returns:
        --------
            np_ac: np.array - control signal
        """
        # calculate the distance between the current robot and the goal position
        dis = self.calc_norm_dis(self.np_curPos, self.np_goal)

        # if the robot is already at the goal position, return zero vector
        if dis < const.E:
            return const.NP_ZERO
        
        # initialize the control signal
        np_ac = np.zeros(2)
        
        # iterate through the list of robots
        for robot in lst_robots:

            if robot.int_index == self.int_index:
                # skip the current robot
                continue
            
            # calculate the relative position and distance between the current robot and the other robot
            rel_dis = self.calc_relative_pos(self.np_curPos, robot.np_curPos)
            dis     = self.calc_norm_dis(self.np_curPos, robot.np_curPos)

            # create the control signal
            np_ac_i = - rel_dis/dis

            # only add the control signal if the other robot is within the sensing radius of current robot
            if dis < const.SS_RANGE:

                # add the control signal to the total control signal
                np_ac += (const.SS_RANGE - dis)/(const.SS_RANGE - const.R_ROBOT - 0.02)*np_ac_i
            
        return np_ac

    def random_walk(self):
        """
        Create control signal for random walk.

        Parameters:
        -----------
            None

        Returns:
        --------
            np_rw: np.array - control signal
        """
        # calculate the distance between the current robot and the goal position
        dis = self.calc_norm_dis(self.np_curPos, self.np_goal)

        # if the robot is already at the goal position, return zero vector
        if dis < const.E:
            return const.NP_ZERO
        
        # create control signal for random walk
        np_rw = np.random.rand(2)
        return np_rw

    @staticmethod
    def calc_norm_dis(pos1: np.array, pos2: np.array):
        """
        Calculate the distance between two points.

        Parameters:
        -----------
            pos1: np.array - position 1
            pos2: np.array - position 2

        Returns:
        --------
            distance: float - distance between the two points
        """
        return norm(pos2 - pos1)

    @staticmethod
    def calc_relative_pos(pos1: np.array, pos2: np.array):
        """
        Calculate the relative position between two points.

        Parameters:
        -----------
            pos1: np.array - position 1
            pos2: np.array - position 2

        Returns:
        --------
            relative_pos: np.array - relative position between the two points
        """
        relative_pos = pos2 - pos1
        return relative_pos