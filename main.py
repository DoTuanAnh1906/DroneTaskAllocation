from const import const
from controller.robot import *
from views import views
from numpy.random import rand
from scipy.optimize import linear_sum_assignment

# create a list of robots
lst_robots: list['Robot'] = []

starts_pos  = rand(const.N, 2) * const.OFFSET
_goals_pos  = rand(const.N, 2) * const.OFFSET + const.OFFSET

# hungarian algorithm to assign the goal to each robot with minimum distance
dx = _goals_pos[:, 0] - starts_pos[:, 0, None]
dy = _goals_pos[:, 1] - starts_pos[:, 1, None]

# calculate the cost matrix
cost_matrix = np.hypot(dx, dy)

# apply the hungarian algorithm
lst_starts_idx, lst_goals_idx = linear_sum_assignment(cost_matrix)

# create the robots with the minimum assigned goals
for i in range(const.N):
    start_pos = starts_pos[i, :]
    goal_pos = _goals_pos[lst_goals_idx[i], :]
    robot = Robot(i, start_pos, goal_pos)
    lst_robots.append(robot)

# set the initial iteration = 1
it = 1

while it < const.NUM_STEP:
    # calculate the control signal for each robot
    lst_controls = []
    for i in range(const.N):
        control_signal = lst_robots[i].calc_control_signal(lst_robots)
        lst_controls.append(control_signal)

    # update the state of each robot
    for i in range(const.N):
        lst_robots[i].update_pos(lst_controls[i], const.DT)

    # check if all robots have reached their goals
    count = 0
    for i in range(const.N):
        dis = np.linalg.norm(lst_robots[i].np_goal - lst_robots[i].np_curPos)
        if dis < const.E:
            count += 1
        else:
            break
    if count == const.N:
        break

    # increment the iteration
    it += 1

# get the iteration number
print(it)

# export the robots position to image

# views.plot_single_robot(robot1)
# views.plot_multiple_robots(lst_robots)
views.plot_step_robots(lst_robots)