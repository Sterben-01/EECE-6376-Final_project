# Author: Ziqi Zhao
# Vanderbilt University
# CS-6376 F21 final project

#---------------------------------------------------------------------------------------------------------------/
# After run this file, you can choose generate a .gif file to default folder or show a .png file at right side.
# If you choose to show the .gif file, comment line 323-325.
#---------------------------------------------------------------------------------------------------------------/


import copy
import pdb
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def flight_step_1(plane_position=None, plane_init=None, plane_dest=None, compel_param=None):
    if compel_param is not None:
        return compel_param
    if plane_position is None:
        plane_position = []
    if plane_init is None:
        plane_init = []
    if plane_dest is None:
        plane_dest = []
    cur_position = copy.deepcopy(plane_position)

    if (cur_position == plane_dest):
        return cur_position

    elif (cur_position[0] < plane_dest[0]):
        cur_position[0] = cur_position[0] + 1

    elif (cur_position[0] > plane_dest[0]):
        cur_position[0] = cur_position[0] - 1

    elif (cur_position[1] < plane_dest[1]):
        cur_position[1] = cur_position[1] + 1

    elif (cur_position[1] > plane_dest[1]):
        cur_position[1] = cur_position[1] - 1
    return cur_position


def flight_step_2(plane_position=None, plane_init=None, plane_dest=None,
                  plane_b_cur_pos=None, plane_b_next_pos=None, compel_param=None, history_path=None):
    if history_path is None:
        history_path = []
    if compel_param is not None:
        return compel_param
    if plane_position is None:
        plane_position = []
    if plane_init is None:
        plane_init = []
    if plane_dest is None:
        plane_dest = []
    if plane_b_cur_pos is None:
        plane_b_cur_pos = []
    if plane_b_next_pos is None:
        plane_b_next_pos = []
    cur_position = copy.deepcopy(plane_position)

    arrive = False

    if (cur_position == plane_dest):
        return cur_position
    elif (cur_position[0] < plane_dest[0]):
        cur_position[0] = cur_position[0] + 1

    elif (cur_position[0] > plane_dest[0]):
        cur_position[0] = cur_position[0] - 1

    elif (cur_position[1] < plane_dest[1]):
        cur_position[1] = cur_position[1] + 1

    elif (cur_position[1] > plane_dest[1]):
        cur_position[1] = cur_position[1] - 1


    while (((len(history_path) >= 2) and (cur_position == history_path[-2])) or collision(True, plane_position, plane_b_cur_pos, cur_position, plane_b_next_pos) == True):
        print("history path", history_path)
        print(cur_position)
        indicator = next_step_direction(plane_position, cur_position)
        if (indicator == 1):
            indicator = 3
        elif (indicator == 2):
            indicator = 4
        elif (indicator == 3):
            indicator = 2
        elif (indicator == 4):
            indicator = 1
        cur_position = turn_goforward(indicator, plane_position)
        print("currentpos", cur_position)

    return cur_position


def next_step_direction(plane_cur_position=None, plane_next_position=None):
    pos_indicator = 0  # 0 default, 1 down, 2 up, 3 left, 4 right, 5 error
    if plane_cur_position is None:
        plane_cur_position = []
    if plane_next_position is None:
        plane_next_position = []
    if (plane_cur_position[0] == plane_next_position[0]):
        if (plane_cur_position[1] > plane_next_position[1]):
            pos_indicator = 1  # down
            print("down")

        if (plane_cur_position[1] < plane_next_position[1]):
            pos_indicator = 2  # up
            print("up")

    if (plane_cur_position[1] == plane_next_position[1]):
        if (plane_cur_position[0] > plane_next_position[0]):
            pos_indicator = 3  # left
            print("left")

        if (plane_cur_position[0] < plane_next_position[0]):
            pos_indicator = 4  # right
            print("right")

    return pos_indicator


def turn_left(next_step_indicator):
    temp_indicator = next_step_indicator
    if (next_step_indicator == 1):
        temp_indicator = 4
    if (next_step_indicator == 2):
        temp_indicator = 3
    if (next_step_indicator == 3):
        temp_indicator = 1
    if (next_step_indicator == 4):
        temp_indicator = 2

    return temp_indicator


def turn_right(next_step_indicator):
    temp_indicator = next_step_indicator
    if (next_step_indicator == 1):
        temp_indicator = 3
    if (next_step_indicator == 2):
        temp_indicator = 4
    if (next_step_indicator == 3):
        temp_indicator = 2
    if (next_step_indicator == 4):
        temp_indicator = 1

    return temp_indicator


def turn_goforward(next_step_indicator, plane_cur_pos=None):
    if plane_cur_pos is None:
        plane_cur_pos = []
    temp_cur_pos = copy.deepcopy(plane_cur_pos)
    if (next_step_indicator == 1):
        temp_cur_pos[1] = temp_cur_pos[1] - 1
    if (next_step_indicator == 2):
        temp_cur_pos[1] = temp_cur_pos[1] + 1
    if (next_step_indicator == 3):
        temp_cur_pos[0] = temp_cur_pos[0] - 1
    if (next_step_indicator == 4):
        temp_cur_pos[0] = temp_cur_pos[0] + 1
    return temp_cur_pos


def communicate(plane_a_cur=None, plane_b_cur=None):
    if (abs(plane_a_cur[0] - plane_b_cur[0]) <= 4 or abs(plane_a_cur[1] - plane_b_cur[1] <= 4)):
        communicateable = True
    else:
        communicateable = False
    return communicateable


def collision(communicateable, aircraft_a_cur_pos=None, aircraft_b_cur_pos=None, aircraft_a_next_pos=None,
              aircraft_b_next_pos=None):
    if aircraft_a_cur_pos is None:
        aircraft_a_cur_pos = []
    if aircraft_b_cur_pos is None:
        aircraft_b_cur_pos = []
    if aircraft_a_next_pos is None:
        aircraft_a_next_pos = []
    if aircraft_b_next_pos is None:
        aircraft_b_next_pos = []
    collision_detect = False
    if (communicateable == True):
        if (aircraft_a_next_pos == aircraft_b_next_pos):
            collision_detect = True
        if ((aircraft_a_next_pos == aircraft_b_cur_pos) and (aircraft_a_cur_pos == aircraft_b_next_pos)):
            collision_detect = True
    if (collision_detect == True):
        print("collision detected")
        print(aircraft_a_cur_pos)
        print(aircraft_b_cur_pos)
        print(aircraft_a_next_pos)
        print(aircraft_b_next_pos)
    return collision_detect


def main():
    global aircraft_a_cur_x, aircraft_a_cur_y, aircraft_b_cur_x, aircraft_b_cur_y, axis

    def draw_frame(i):
        global aircraft_a_cur_x, aircraft_a_cur_y, aircraft_b_cur_x, aircraft_b_cur_y, axis

        ax = aircraft_a_cur_x[:i+1]
        ay = aircraft_a_cur_y[:i+1]
        bx = aircraft_b_cur_x[:i+1]
        by = aircraft_b_cur_y[:i+1]

        axis.plot(ax, ay, alpha=0.5, c='r', marker='o')
        axis.plot(bx, by, alpha=0.5, c='b', marker='*')

    fig, axis = plt.subplots()

    aircraft_a_init = [2, 2]  # change this to modify aircraft A initial point
    aircraft_a_dest = [7, 6]  # change this to modify aircraft A destination point
    aircraft_b_init = [1, 3]  # change this to modify aircraft B initial point
    aircraft_b_dest = [8, 6]  # change this to modify aircraft B destination point
    aircraft_a_cur_x = []
    aircraft_a_cur_y = []
    aircraft_b_cur_x = []
    aircraft_b_cur_y = []
    communicateable = False
    aircraft_a_cur = aircraft_a_init
    aircraft_b_cur = aircraft_b_init
    collision_detect_a = False
    collision_detect_b = False
    tick_list = []
    for i in range(0, 2):
        tick_list.append(aircraft_a_init[i])
        tick_list.append(aircraft_a_dest[i])
        tick_list.append(aircraft_b_init[i])
        tick_list.append(aircraft_b_dest[i])
    my_x_ticks = np.arange(0, max(tick_list) + 3, 1)
    plt.xlim((0, max(tick_list)))
    plt.ylim((0, max(tick_list)))
    plt.xticks(my_x_ticks)
    plt.yticks(my_x_ticks)
    aircraft_a_cur_x.append(aircraft_a_cur[0])
    aircraft_a_cur_y.append(aircraft_a_cur[1])
    aircraft_b_cur_x.append(aircraft_b_cur[0])
    aircraft_b_cur_y.append(aircraft_b_cur[1])
    aircraft_a_com_cur = [aircraft_a_cur]
    a_new_next = None
    # if(aircraft_b_cur == aircraft_b_dest):
    #     while(aircraft_a_cur != aircraft_a_dest):
    #         aircraft_a_cur = flight_step_1(aircraft_a_cur, aircraft_a_init, aircraft_a_dest)
    #         aircraft_a_cur_x.append(aircraft_a_cur[0])
    #         aircraft_a_cur_y.append(aircraft_a_cur[1])
    #
    # if(aircraft_a_cur == aircraft_a_dest):
    #     while(aircraft_b_cur != aircraft_b_dest):
    #         aircraft_b_cur = flight_step_1(aircraft_b_cur, aircraft_b_init, aircraft_b_dest)
    #         aircraft_b_cur_x.append(aircraft_b_cur[0])
    #         aircraft_b_cur_y.append(aircraft_b_cur[1])
    # else:
    while ((aircraft_a_cur != aircraft_a_dest) or (aircraft_b_cur != aircraft_b_dest)):
        if(aircraft_a_cur == aircraft_a_dest):
            while (aircraft_b_cur != aircraft_b_dest):
                aircraft_b_cur = flight_step_1(aircraft_b_cur, aircraft_b_init, aircraft_b_dest)
                aircraft_b_cur_x.append(aircraft_b_cur[0])
                aircraft_b_cur_y.append(aircraft_b_cur[1])
            break
        if(aircraft_b_cur == aircraft_b_dest):
            while (aircraft_a_cur != aircraft_a_dest):
                aircraft_a_cur = flight_step_1(aircraft_a_cur, aircraft_a_init, aircraft_a_dest)
                aircraft_a_cur_x.append(aircraft_a_cur[0])
                aircraft_a_cur_y.append(aircraft_a_cur[1])
            break
        communicateable = communicate(aircraft_a_cur, aircraft_b_cur)
        # if aircraft_a_cur == [7, 2]:
        #     pdb.set_trace()
        b_next = flight_step_1(aircraft_b_cur, aircraft_b_init, aircraft_b_dest)
        aircraft_a_cur = flight_step_2(aircraft_a_cur, aircraft_a_init, aircraft_a_dest, aircraft_b_cur, b_next,
                                       a_new_next, aircraft_a_com_cur)  # a move 1 step
        aircraft_b_cur = flight_step_1(aircraft_b_cur, aircraft_b_init, aircraft_b_dest)  # b move 1 step

        a_next = flight_step_1(aircraft_a_cur, aircraft_a_init, aircraft_a_dest)

        b_next = flight_step_1(aircraft_b_cur, aircraft_b_init, aircraft_b_dest)
        print("jkkjk")
        print('current a', aircraft_a_cur)
        print('current b', aircraft_b_cur)
        aircraft_a_cur_x.append(aircraft_a_cur[0])
        aircraft_a_cur_y.append(aircraft_a_cur[1])
        aircraft_b_cur_x.append(aircraft_b_cur[0])
        aircraft_b_cur_y.append(aircraft_b_cur[1])
        aircraft_a_com_cur.append(aircraft_a_cur)
        # calc a move 2 step( move 1 step based on line 188)

        print('next a', a_next)

        print("next b", b_next)

        next_step_indicator_a = next_step_direction(aircraft_a_cur, a_next)
        print("****")
        print(next_step_indicator_a)
        collision_detect_a = collision(communicateable, aircraft_a_cur, aircraft_b_cur, a_next, b_next)
        if((aircraft_a_cur == aircraft_a_dest) or (aircraft_b_cur == aircraft_b_dest)):
            collision_detect_a == False
        if (collision_detect_a == True):
            after_col_cal_next_step_indicator_a = turn_right(next_step_indicator_a)
            a_new_next = turn_goforward(after_col_cal_next_step_indicator_a, aircraft_a_cur)
            print("^^^^")
            print(aircraft_a_cur)
            print(after_col_cal_next_step_indicator_a)
            print(aircraft_a_cur_x)
            print(aircraft_a_cur_y)
            print(a_new_next)
            if ((a_new_next[0] == aircraft_a_cur_x[-2]) and (a_new_next[1] == aircraft_a_cur_y[-2])):
                after_after_col_cal_next_step_indicator_a = turn_right(after_col_cal_next_step_indicator_a)
                print("#####")
                print(after_after_col_cal_next_step_indicator_a)
                a_new_next = turn_goforward(after_after_col_cal_next_step_indicator_a, aircraft_a_cur)
                print(a_new_next)
        else:
            a_new_next = None
            print("---------------------------")
    plt.plot(aircraft_a_cur_x, aircraft_a_cur_y, alpha=0.5, c='r', marker='o') # If you choose to show the .gif file, comment this line
    plt.plot(aircraft_b_cur_x, aircraft_b_cur_y, alpha=0.5, c='b', marker='*') # If you choose to show the .gif file, comment this line
    plt.show() # If you choose to show the .gif file, comment this line
    print(aircraft_a_cur_x)
    print(aircraft_a_cur_y)
    print(aircraft_b_cur_x)
    print(aircraft_b_cur_y)
    anim = animation.FuncAnimation(fig=fig,
                                   func=draw_frame,
                                   frames=np.arange(0, len(aircraft_a_cur_x), 1),
                                   interval=1000)
    anim.save('mainoutp.gif')


if __name__ == "__main__":
    main()



