from const import const
from controller.robot import *
from controller import export as exp

# create a list of robots
lst_robots: list['Robot'] = []

for i in range(const.N):
    init_pos = np.random.rand(2) * const.CR
    init_goal = np.random.rand(2) * const.CR - const.CR
    robot = Robot(i, init_pos, init_goal)
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