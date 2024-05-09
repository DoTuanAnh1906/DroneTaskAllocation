from const import const
from controller.robot import *
from views import views

# create a list of robots
lst_robots: list['Robot'] = []

lst_goals_pos = []
lst_start_pos = []

robot1 = Robot(0, np.array([0, 0]), np.array([10, 10]))
lst_robots.append(robot1)
robot2 = Robot(1, np.array([10, 10]), np.array([0, 0]))
lst_robots.append(robot2)
robot3 = Robot(2, np.array([10, 0]), np.array([0, 10]))
lst_robots.append(robot3)
robot4 = Robot(3, np.array([0, 10]), np.array([10, 0]))
lst_robots.append(robot4)

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
views.plot_multiple_robots(lst_robots)
# views.plot_step_robots(lst_robots)